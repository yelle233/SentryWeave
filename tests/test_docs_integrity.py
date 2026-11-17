"""Verify every public Markdown link, heading anchor, media asset and bridge snippet."""

from pathlib import Path
import re
from urllib.parse import unquote

from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
FENCE_RE = re.compile(r"^(`{3,}|~{3,})")
MD_LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
HTML_REF_RE = re.compile(r"""(?:src|href)=["']([^"']+)["']""")
HEADING_RE = re.compile(r"^(#{1,6})\s+(.+?)\s*$")
PY_BLOCK_RE = re.compile(r"```python\n(.*?)```", re.DOTALL)


def public_markdown_files() -> list[Path]:
    files = [ROOT / "README.md", ROOT / "CONTRIBUTING.md", ROOT / "THIRD_PARTY_NOTICES.md"]
    files.extend(sorted(p for p in (ROOT / "docs").rglob("*.md")
                        if "superpowers" not in p.parts))
    return files


def strip_fences(text: str) -> str:
    out, marker = [], None
    for line in text.splitlines():
        match = FENCE_RE.match(line.strip())
        if match:
            token = match.group(1)
            if marker is None:
                marker = token[0] * len(token)
            elif line.strip().startswith(marker[0]) and len(token) >= len(marker):
                marker = None
            continue
        if marker is None:
            out.append(line)
    return "\n".join(out)


def github_slug(heading: str) -> str:
    clean = re.sub(r"`([^`]*)`", r"\1", heading)
    clean = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", clean).strip().lower()
    clean = re.sub(r"[^\w\u4e00-\u9fff\- ]+", "", clean)
    return clean.replace(" ", "-")


def headings_of(md_file: Path) -> set[str]:
    slugs: set[str] = set()
    counts: dict[str, int] = {}
    for line in strip_fences(md_file.read_text(encoding="utf-8")).splitlines():
        match = HEADING_RE.match(line)
        if not match:
            continue
        base = github_slug(match.group(2))
        seen = counts.get(base, 0)
        counts[base] = seen + 1
        slugs.add(base if seen == 0 else f"{base}-{seen}")
    return slugs


def test_all_relative_markdown_and_html_links_resolve():
    missing = []
    for doc in public_markdown_files():
        body = strip_fences(doc.read_text(encoding="utf-8"))
        refs = [*MD_LINK_RE.findall(body), *HTML_REF_RE.findall(body)]
        for raw in refs:
            if raw.startswith(("http://", "https://", "mailto:")):
                continue
            path_part, _, anchor = raw.partition("#")
            target = (doc.parent / unquote(path_part)).resolve() if path_part else doc.resolve()
            if not target.exists():
                missing.append(f"{doc.relative_to(ROOT)} -> {raw} (missing file)")
                continue
            if anchor and target.is_file() and target.suffix == ".md":
                if unquote(anchor).lower() not in headings_of(target):
                    missing.append(f"{doc.relative_to(ROOT)} -> {raw} (missing #{anchor})")
    assert not missing, "\n".join(missing)


def test_readme_required_media_and_assets_are_valid():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for token in (
        "# SentryWeave",
        "未来战队",
        "Apache",
        "https://github.com/RM-Arena/Arena-1",
        "goal_scope",
        "SentryWeave 是独立项目，不代表 RoboMaster 官方规则、认证或背书。",
    ):
        assert token in readme, f"README.md missing protected text: {token}"
    required = [
        "docs/assets/logo.png",
        "docs/assets/performance.png",
        "docs/media/support-push.gif",
        "docs/media/intercept-raid.gif",
        "docs/media/low-hp-retreat.gif",
        "docs/media/radar-missing.gif",
    ]
    for rel in required:
        assert rel in readme, f"README.md missing reference to {rel}"
        path = ROOT / rel
        assert path.is_file(), f"Missing asset {rel}"
        size = path.stat().st_size
        assert size > 1024, f"Asset too small: {rel}"
        if path.suffix == ".gif":
            assert size <= 5 * 1024 * 1024, f"GIF exceeds 5 MiB: {rel}"
        with Image.open(path) as img:
            assert img.width >= 100 and img.height >= 100
            if path.suffix == ".gif":
                assert getattr(img, "n_frames", 1) >= 10


def test_python_code_blocks_in_new_guides_compile():
    docs = [
        ROOT / "docs/interfaces.md",
        ROOT / "docs/navigation-integration.md",
        ROOT / "docs/real-robot.md",
        ROOT / "README.md",
    ]
    for doc in docs:
        blocks = PY_BLOCK_RE.findall(doc.read_text(encoding="utf-8"))
        assert blocks, f"Expected at least one python example in {doc.name}"
        for idx, code in enumerate(blocks):
            compile(code, f"{doc.name}#block{idx}", "exec")
