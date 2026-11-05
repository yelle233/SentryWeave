"""Supervised HMM fitting/calibration and closed-loop utility weight search."""

from dataclasses import replace
import json
from pathlib import Path
import platform

import numpy as np

from sentry_core.config import Config
from sentry_core.intent import FEATURE_NAMES, FEATURE_VERSION
from sentry_core.learned import IntentModel
from sentry_core.model import IntentType
from .dataset import split_sequences


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def classification_metrics(model, sequences):
    labels, predictions = [], []
    for sequence in sequences:
        posterior = None
        for x, label in zip(sequence.features, sequence.labels):
            posterior = model.posterior(x, posterior)
            predictions.append(posterior)
            labels.append(int(label))
    y, probability = np.asarray(labels), np.asarray(predictions)
    predicted = probability.argmax(axis=1)
    confidence = probability.max(axis=1)
    correct = predicted == y
    confusion = np.zeros((6, 6), dtype=int)
    np.add.at(confusion, (y, predicted), 1)
    f1 = []
    for i in range(6):
        tp = confusion[i, i]
        f1.append(2 * tp / max(1, confusion[i].sum() + confusion[:, i].sum()))
    ece = 0.
    for left, right in zip(np.linspace(0, 1, 11)[:-1], np.linspace(0, 1, 11)[1:]):
        mask = (confidence > left) & (confidence <= right)
        if mask.any():
            ece += float(mask.mean() * abs(correct[mask].mean() - confidence[mask].mean()))
    return {
        "samples": len(y), "accuracy": float(correct.mean()), "macro_f1": float(np.mean(f1)),
        "nll": float(-np.log(np.maximum(probability[np.arange(len(y)), y], 1e-12)).mean()),
        "brier": float(np.mean(np.sum((probability - np.eye(6)[y]) ** 2, axis=1))),
        "ece_10_bins": ece, "confusion_matrix": confusion.tolist(),
    }


def train_intent(sequences, config: Config, output_path, seed=17):
    if any(s.map_fingerprint != config.map_fingerprint for s in sequences):
        raise ValueError("dataset map does not match configuration")
    training, validation, test = split_sequences(sequences, seed)
    x = np.concatenate([s.features for s in training])
    y = np.concatenate([s.labels for s in training])
    means, variances = [], []
    for label in IntentType:
        samples = x[y == int(label)]
        if len(samples) < 2:
            raise ValueError(f"training data has fewer than two {label.name} observations")
        means.append(samples.mean(axis=0))
        variances.append(np.maximum(samples.var(axis=0), 0.015))
    transition = np.ones((6, 6)) + np.eye(6) * 5
    prior = np.ones(6)
    for sequence in training:
        prior[sequence.labels[0]] += 1
        np.add.at(transition, (sequence.labels[:-1], sequence.labels[1:]), 1)
    payload = {
        "artifact_type": "sentry_intent_hmm", "schema_version": 1,
        "feature_version": FEATURE_VERSION, "feature_names": list(FEATURE_NAMES),
        "labels": [label.name for label in IntentType], "map_fingerprint": config.map_fingerprint,
        "means": np.asarray(means).tolist(), "variances": np.asarray(variances).tolist(),
        "transition": (transition / transition.sum(axis=1, keepdims=True)).tolist(),
        "prior": (prior / prior.sum()).tolist(), "temperature": 1.,
        "training": {"seed": seed, "sources": sorted({s.source for s in training}),
                     "python_version": platform.python_version(), "numpy_version": np.__version__,
                     "train_groups": sorted({s.group_id for s in training})},
    }
    calibration = []
    for temperature in (0.5, 1., 2., 4., 8., 12., 16.):
        trial = {**payload, "temperature": temperature}
        metrics = classification_metrics(IntentModel(trial, config), validation)
        calibration.append((metrics["nll"], temperature))
    payload["temperature"] = min(calibration)[1]
    model = IntentModel(payload, config)
    report = {
        "schema_version": 1, "split_seed": seed,
        "splits": {name: sorted({s.group_id for s in data}) for name, data in
                   (("train", training), ("validation", validation), ("test", test))},
        "temperature": payload["temperature"],
        "validation": classification_metrics(model, validation),
        "test": classification_metrics(model, test),
        "limitations": "Synthetic labels measure generator generalization, not real match intent.",
    }
    write_json(output_path, payload)
    write_json(Path(output_path).with_suffix(".report.json"), report)
    return report


def tune_utility(config: Config, iterations=6, population=16,
                 scenarios=("team_push", "base_raid", "conflict", "low_hp", "dropout"),
                 duration=30., seed=31, model=None):
    from .simulation import rollout

    if iterations < 1 or population < 4 or duration <= 0 or not scenarios:
        raise ValueError("invalid CEM search size")
    rng = np.random.default_rng(seed)
    train_seeds = [seed * 100 + i for i in range(2)]
    validation_seeds = [seed * 100 + 50 + i for i in range(2)]

    def score(weights, seeds):
        trial = replace(config, weights=tuple(float(w) for w in weights))
        return float(np.mean([rollout(trial, scenario, s, duration=duration, model=model,
                                      policy="learned" if model is not None else "rule")["reward"]
                              for scenario in scenarios for s in seeds]))

    mean, std = np.log(np.maximum(config.weights, 1e-3)), np.full(6, 0.8)
    best_weights = np.asarray(config.weights)
    baseline_score = score(best_weights, validation_seeds)
    best_score = baseline_score
    history = []
    for iteration in range(iterations):
        logits = rng.normal(mean, std, size=(population, 6))
        logits[0] = np.log(np.maximum(best_weights, 1e-8))
        weight_sets = np.exp(logits - logits.max(axis=1, keepdims=True))
        weight_sets /= weight_sets.sum(axis=1, keepdims=True)
        scores = np.asarray([score(w, train_seeds) for w in weight_sets])
        elite = np.argsort(scores)[-max(2, population // 4):]
        mean = 0.3 * mean + 0.7 * logits[elite].mean(axis=0)
        std = np.maximum(0.15, logits[elite].std(axis=0))
        candidate = weight_sets[int(scores.argmax())]
        validation_score = score(candidate, validation_seeds)
        if validation_score > best_score:
            best_score, best_weights = validation_score, candidate
        history.append({"iteration": iteration, "train_best": float(scores.max()),
                        "validation_candidate": validation_score, "validation_best": best_score})
    return {
        "artifact_type": "sentry_utility", "schema_version": 1,
        "map_fingerprint": config.map_fingerprint, "weights": best_weights.tolist(),
        "intent_source": "hmm" if model is not None else "rule",
        "seed": seed, "train_seeds": train_seeds, "validation_seeds": validation_seeds,
        "scenarios": list(scenarios), "duration": duration,
        "baseline_validation_reward": baseline_score, "validation_reward": best_score,
        "history": history,
        "limitations": "Kinematic navigation and coverage surrogate; evaluate on unseen match seeds.",
    }
