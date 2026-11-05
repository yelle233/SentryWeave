#!/usr/bin/env python3
"""Import pinned PolarBear navigation/maps and Fortress static arenas for Jazzy.

Original checkouts are read-only. The output is a separate, COLCON_IGNORE'd
asset tree consumed by sentry_bringup, not an untested build of old C++ plugins.
"""

import argparse
import copy
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import yaml


NAV_COMMIT = "9c43e38a902ed21df1960491dcc8423263599f0f"
SIM_COMMIT = "c1ca66f2dbb542d36cd8452dce394ce989ec5225"
ORGANIZATION = "https://github.com/SMBU-PolarBear-Robotics-Team"
ROOT = Path(__file__).resolve().parents[1]
SYSTEMS = {
    "Physics", "UserCommands", "SceneBroadcaster", "Sensors", "Imu",
    "PerformerDetector", "Contact", "PosePublisher", "OdometryPublisher",
}


def migrate_sdf(text: str, spawn=None) -> str:
    root = ET.fromstring(text, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    root.set("version", "1.9")
    for element in root.iter():
        if element.get("frame") == "":
            del element.attrib["frame"]
    for plugin in root.iter("plugin"):
        name = plugin.get("name", "")
        if "::systems::" not in name or name.rsplit("::", 1)[-1] not in SYSTEMS:
            raise ValueError(f"custom Gazebo plugin requires a source-level port: {name}")
        plugin.set("name", name.replace("ignition::gazebo::", "gz::sim::"))
        filename = plugin.get("filename", "")
        filename = re.sub(r"^(lib)?ignition-gazebo-", "gz-sim-", filename)
        if filename.endswith(".so"):
            filename = filename[:-3]
        plugin.set("filename", filename)
    world = root.find("world")
    if spawn is not None and world is not None:
        if any(m.get("name") == "sentry" for m in world.findall("model")):
            raise ValueError("source world already contains a sentry")
        include = ET.SubElement(world, "include")
        ET.SubElement(include, "uri").text = "model://sentry"
        ET.SubElement(include, "name").text = "sentry"
        ET.SubElement(include, "pose").text = f"{spawn[0]} {spawn[1]} 0.05 0 0 {spawn[2]}"
    ET.indent(root, space="  ")
    notice = "<!-- Modified by RMJC for Gazebo Harmonic; source revision: import_manifest.json. -->\n"
    return '<?xml version="1.0"?>\n' + notice + ET.tostring(root, encoding="unicode") + "\n"


def migrate_bridge(data):
    result = copy.deepcopy(data)
    if not isinstance(result, list):
        raise ValueError("bridge configuration must be a list")
    for entry in result:
        if not isinstance(entry.get("gz_type_name"), str):
            raise ValueError("bridge entry missing gz_type_name")
        entry["gz_type_name"] = entry["gz_type_name"].replace("ignition.msgs.", "gz.msgs.")
    return result


def migrate_navigation(source, reference):
    # Reuse compatible standard Nav2 settings; custom plugins get explicit, documented replacements.
    result = {key: copy.deepcopy(source.get(key, value)) for key, value in reference.items()}
    replaced = ["pb_omni_pid_pursuit_controller -> Jazzy reference controller",
                "pb_nav2_costmap_2d::IntensityVoxelLayer -> standard LaserScan obstacle layer",
                "pb_nav2_behaviors::BackUpFreeSpace -> nav2_behaviors::BackUp",
                "LIO / gimbal fake frame -> reference base_link / odom Interface"]
    for key in ("controller_server", "behavior_server", "velocity_smoother", "collision_monitor"):
        result[key] = copy.deepcopy(reference[key])
    for key in ("local_costmap", "global_costmap"):
        # Original intensity semantics cannot be retained by renaming its plugin.
        result[key] = copy.deepcopy(reference[key])
    navigator = result["bt_navigator"]["ros__parameters"]
    plugins = navigator.pop("plugin_lib_names", [])
    custom = [p for p in plugins if not p.startswith("nav2_")]
    if custom:
        raise ValueError(f"custom BT libraries need an explicit Jazzy port: {custom}")
    navigator.pop("default_nav_to_pose_bt_xml", None)
    navigator.pop("default_nav_through_poses_bt_xml", None)
    for key in ("navigators", "navigate_to_pose", "error_code_names"):
        navigator[key] = copy.deepcopy(reference["bt_navigator"]["ros__parameters"][key])

    def walk(value):
        if isinstance(value, dict):
            for key, item in list(value.items()):
                if key == "plugin" and isinstance(item, str) and item.startswith("nav2_"):
                    value[key] = item.replace("/", "::")
                elif key in ("robot_base_frame", "base_frame_id"):
                    value[key] = "base_link"
                elif key == "odom_topic":
                    value[key] = "odom"
                else:
                    walk(item)
        elif isinstance(value, list):
            for item in value:
                walk(item)
    walk(result)
    return result, {
        "replaced_modules": replaced,
        "retained": ["standard global planner settings", "map_server settings", "BT timing settings"],
        "coordinate_contract": "Calibrate world_to_map and tactical points before using upstream maps.",
    }


def _revision(path, expected):
    actual = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
    if actual != expected:
        raise ValueError(f"upstream revision mismatch at {path}: expected {expected}, got {actual}")
    dirty = subprocess.check_output(["git", "-C", str(path), "status", "--porcelain"], text=True)
    if dirty:
        raise ValueError(f"upstream checkout is modified: {path}; use clean pinned sources")


def fetch_sources(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    paths = []
    for name, revision in (("pb2025_sentry_nav", NAV_COMMIT), ("rmu_gazebo_simulator", SIM_COMMIT)):
        path = directory / name
        if not path.exists():
            subprocess.run(["git", "init", str(path)], check=True)
            subprocess.run(["git", "-C", str(path), "remote", "add", "origin",
                            f"{ORGANIZATION}/{name}.git"], check=True)
            subprocess.run(["git", "-C", str(path), "fetch", "--depth", "1", "origin", revision],
                           check=True)
            subprocess.run(["git", "-C", str(path), "checkout", "--detach", "FETCH_HEAD"], check=True)
        _revision(path, revision)
        paths.append(path)
    (directory / "COLCON_IGNORE").touch()
    return paths


def import_assets(nav, simulator, destination, spawn=(3., 6., 0.)):
    nav, simulator, destination = Path(nav), Path(simulator), Path(destination)
    _revision(nav, NAV_COMMIT)
    _revision(simulator, SIM_COMMIT)
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {destination}")
    source_resources = simulator / "rmu_gazebo_simulator/resource"
    source_maps = nav / "pb2025_nav_bringup/map/simulation"
    for yaml_file in source_maps.glob("*.yaml"):
        image = yaml.safe_load(yaml_file.read_text())["image"]
        if not (yaml_file.parent / image).is_file():
            raise ValueError(f"map image missing in sparse/LFS checkout: {image}")
    for model in (source_resources / "models").glob("*/model.sdf"):
        for uri in ET.fromstring(model.read_text()).iter("uri"):
            if uri.text and uri.text.startswith("model://"):
                asset = source_resources / "models" / uri.text.removeprefix("model://")
                if not asset.exists():
                    raise ValueError(f"model asset missing in sparse/LFS checkout: {asset}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".rmjc-import-", dir=destination.parent) as temporary:
        output = Path(temporary) / "assets"
        output.mkdir()
        shutil.copytree(source_resources / "models", output / "models")
        shutil.copytree(source_maps, output / "maps")
        (output / "worlds").mkdir()
        for world in (source_resources / "worlds").glob("*.sdf"):
            (output / "worlds" / world.name).write_text(migrate_sdf(world.read_text(), spawn))
        for model in (output / "models").glob("*/model.sdf"):
            model.write_text(migrate_sdf(model.read_text()))
        reference = yaml.safe_load((ROOT / "src/sentry_bringup/config/nav2.yaml").read_text())
        source = yaml.safe_load((nav / "pb2025_nav_bringup/config/simulation/nav2_params.yaml").read_text())
        migrated, audit = migrate_navigation(source, reference)
        notice = "# Modified by RMJC for ROS 2 Jazzy; sources and replacements: import_manifest.json.\n"
        (output / "nav2_jazzy.yaml").write_text(notice + yaml.safe_dump(migrated, sort_keys=False))
        bridge = yaml.safe_load((simulator / "rmu_gazebo_simulator/config/ros_gz_bridge.yaml").read_text())
        (output / "upstream_bridge_jazzy.yaml").write_text(
            notice + yaml.safe_dump(migrate_bridge(bridge), sort_keys=False)
        )
        for source_path, filename in ((nav, "LICENSE.pb2025"), (simulator, "LICENSE.rmu")):
            candidates = [p for p in source_path.glob("LICENSE*") if p.is_file()]
            if not candidates:
                raise ValueError(f"upstream license missing: {source_path}")
            shutil.copy2(candidates[0], output / filename)
            if (source_path / "NOTICE").is_file():
                shutil.copy2(source_path / "NOTICE", output / filename.replace("LICENSE", "NOTICE"))
        manifest = {
            "schema_version": 1, "nav_commit": NAV_COMMIT, "sim_commit": SIM_COMMIT,
            "spawn_world": list(spawn), **audit,
            "scope": "Static arenas, maps, standard Nav2 configuration; custom match/weapon plugins excluded.",
        }
        (output / "import_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        (output / "COLCON_IGNORE").touch()
        output.rename(destination)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fetch", action="store_true", help="download pinned sources into third_party/sources")
    parser.add_argument("--sources", default=str(ROOT / "third_party/sources"))
    parser.add_argument("--nav")
    parser.add_argument("--sim")
    parser.add_argument("--out", default=str(ROOT / "third_party/polarbear_jazzy"))
    parser.add_argument("--spawn", nargs=3, type=float, default=(3., 6., 0.), metavar=("X", "Y", "YAW"))
    args = parser.parse_args()
    if args.fetch:
        nav, simulator = fetch_sources(args.sources)
    elif args.nav and args.sim:
        nav, simulator = args.nav, args.sim
    else:
        parser.error("supply --fetch or both --nav and --sim")
    print(json.dumps(import_assets(nav, simulator, args.out, args.spawn), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
