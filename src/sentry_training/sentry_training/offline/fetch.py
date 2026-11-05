"""Download only the publicly linked official training archive, with pinned checksums."""

import hashlib
import json
from pathlib import Path
import subprocess

from .data import file_hash


ARTICLE = "https://bbs.robomaster.com/article/1936220"
URL = ("https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/"
       "389ecbb646434455af545a78bd2926fe1784290926981/rmuc_2026_region_dataset.7z")
ARCHIVE_BYTES = 113419434
ARCHIVE_MD5 = "89ee19d15db4bfc79094e89419e09189"
DATABASE_SHA256 = "53ac64efeaf570c0e21d8c3724dcefcb0123196b3c7ef963183a911863f724a7"


def fetch(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    archive = directory / "rmuc_2026_region_dataset.7z"
    if not archive.exists():
        temporary = archive.with_suffix(".7z.part")
        subprocess.run(["curl", "-fL", "--retry", "3", "--connect-timeout", "20", "--max-time", "1800",
                        "--continue-at", "-", "--output", str(temporary), URL], check=True)
        digest = hashlib.md5(temporary.read_bytes()).hexdigest()
        if temporary.stat().st_size != ARCHIVE_BYTES or digest != ARCHIVE_MD5:
            raise ValueError("official archive checksum mismatch")
        temporary.rename(archive)
    if archive.stat().st_size != ARCHIVE_BYTES or hashlib.md5(archive.read_bytes()).hexdigest() != ARCHIVE_MD5:
        raise ValueError("existing archive does not match the pinned official source")
    database = directory / "extracted/rmuc_2026_region_dataset.sqlite"
    if not database.exists():
        import py7zr
        with py7zr.SevenZipFile(archive) as bundle:
            if bundle.getnames() != ["rmuc_2026_region_dataset.sqlite"]:
                raise ValueError("unexpected archive members")
            bundle.extractall(database.parent)
    if file_hash(database) != DATABASE_SHA256:
        raise ValueError("extracted official SQLite checksum mismatch")
    manifest = {"article": ARTICLE, "url": URL, "archive_md5": ARCHIVE_MD5,
                "database_sha256": DATABASE_SHA256, "database": str(database),
                "usage": "officially published for team strategy training; not rules/award evidence"}
    (directory / "source.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    return manifest
