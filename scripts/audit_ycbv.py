#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""Check every YCB-V asset's declared frame against the pinned BOP model it claims to be.

For each asset under ``objects/ycbv/``, the BOP model named by ``dataset.ycbv.bop_id`` is
converted from millimetres to metres, mapped into the asset frame by the declared
``frames.T_asset_bop``, and compared with the asset's visual mesh by surface distance.
ICP started from the declared transform then reports how far the best fit lies from it.
The script exits non-zero if any asset exceeds the tolerances below, and writes the
measurements as JSON (default ``docs/ycbv_audit.json``).

Usage:
    uv run --extra dev python scripts/audit_ycbv.py [--archive ycbv_models.zip] [--out PATH]

Without ``--archive`` the pinned archive (525 MB) is downloaded once into
``~/.cache/prl_assets/`` and verified against its SHA-256 before use.
"""

import argparse
import hashlib
import json
import os
import platform
import sys
import tempfile
import urllib.request
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import numpy as np
import scipy
import trimesh
import yaml
from scipy.spatial import cKDTree

ARCHIVE_URL = (
    "https://huggingface.co/datasets/bop-benchmark/ycbv/resolve/"
    "5c2c4aa229800355648cd268040aa814f8dc94f0/ycbv_models.zip"
)
ARCHIVE_SHA256 = "1e517a9b513cadb8eaf406ac3ce3c5f1d6fc4a80161c866b4e2a73d13de5e554"

# BOP ycbv object id -> original YCB model name. BOP ids are contiguous; they are not
# the numeric prefixes of the YCB names.
BOP_YCB_NAMES = {
    1: "002_master_chef_can",
    2: "003_cracker_box",
    3: "004_sugar_box",
    4: "005_tomato_soup_can",
    5: "006_mustard_bottle",
    6: "007_tuna_fish_can",
    7: "008_pudding_box",
    8: "009_gelatin_box",
    9: "010_potted_meat_can",
    10: "011_banana",
    11: "019_pitcher_base",
    12: "021_bleach_cleanser",
    13: "024_bowl",
    14: "025_mug",
    15: "035_power_drill",
    16: "036_wood_block",
    17: "037_scissors",
    18: "040_large_marker",
    19: "051_large_clamp",
    20: "052_extra_large_clamp",
    21: "061_foam_brick",
}

# Two remeshings of one scan differ by about a millimetre; a wrong transform or a
# different object differs by centimetres.
SAMPLES = 20000
MAX_P99_MM = 3.0
MAX_RESIDUAL_DEG = 0.5
MAX_RESIDUAL_MM = 1.0

ROOT = Path(__file__).resolve().parent.parent
YCBV_DIR = ROOT / "src" / "prl_assets" / "objects" / "ycbv"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch_archive(cache: Path) -> Path:
    """Download the pinned archive into ``cache`` unless a verified copy is already there."""
    path = cache / "ycbv_models.zip"
    if path.is_file() and sha256(path) == ARCHIVE_SHA256:
        return path
    cache.mkdir(parents=True, exist_ok=True)
    fd, part = tempfile.mkstemp(dir=cache, suffix=".part")
    try:
        with os.fdopen(fd, "wb") as out, urllib.request.urlopen(ARCHIVE_URL) as r:
            for chunk in iter(lambda: r.read(1 << 20), b""):
                out.write(chunk)
        if sha256(Path(part)) != ARCHIVE_SHA256:
            raise SystemExit(f"Downloaded archive does not match the pinned SHA-256 {ARCHIVE_SHA256}")
        os.replace(part, path)
    finally:
        if os.path.exists(part):
            os.unlink(part)
    return path


def visual_mesh(obj_dir: Path, meta: dict) -> Path:
    """The file of the asset's visual mesh geom, which must sit at the body origin."""
    tree = ET.parse(obj_dir / meta["mujoco"]["xml_path"])
    meshes = {m.get("name"): m.get("file") for m in tree.iter("mesh")}
    visual = [g for g in tree.iter("geom") if g.get("type") == "mesh" and g.get("group") == "2"]
    if len(visual) != 1:
        raise SystemExit(f"{obj_dir.name}: expected one visual mesh geom (group 2), found {len(visual)}")
    if visual[0].get("pos") or visual[0].get("quat") or visual[0].get("euler"):
        raise SystemExit(f"{obj_dir.name}: the visual geom must have no pos/quat/euler offset")
    path = (obj_dir / meshes[visual[0].get("mesh")]).resolve()
    if not path.is_relative_to(obj_dir.resolve()):
        raise SystemExit(f"{obj_dir.name}: visual mesh {path} leaves the asset directory")
    return path


def rigid(matrix: list) -> np.ndarray:
    T = np.asarray(matrix, dtype=float)
    R = T[:3, :3]
    ok = (
        T.shape == (4, 4)
        and np.allclose(T[3], [0, 0, 0, 1])
        and np.allclose(R @ R.T, np.eye(3), atol=1e-6)
        and np.isclose(np.linalg.det(R), 1.0, atol=1e-6)
    )
    if not ok:
        raise SystemExit(f"T_asset_bop is not a rigid transform: {matrix}")
    return T


def icp(src: np.ndarray, dst: np.ndarray, T: np.ndarray, iterations: int = 100) -> np.ndarray:
    tree = cKDTree(dst)
    R, t = T[:3, :3].copy(), T[:3, 3].copy()
    for _ in range(iterations):
        p = src @ R.T + t
        q = dst[tree.query(p)[1]]
        mp, mq = p.mean(0), q.mean(0)
        U, _, Vt = np.linalg.svd((p - mp).T @ (q - mq))
        D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
        dR = Vt.T @ D @ U.T
        R, t = dR @ R, dR @ (t - mp) + mq
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = R, t
    return out


def distances_mm(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return 1e3 * np.concatenate([cKDTree(b).query(a)[0], cKDTree(a).query(b)[0]])


def audit(archive: zipfile.ZipFile, workdir: Path, obj_dir: Path, info: dict) -> dict:
    meta = yaml.safe_load((obj_dir / "meta.yaml").read_text())
    bop_id = meta["dataset"]["ycbv"]["bop_id"]
    if BOP_YCB_NAMES.get(bop_id) != meta["dataset"]["ycbv"]["ycb_name"]:
        raise SystemExit(f"{obj_dir.name}: bop_id {bop_id} is {BOP_YCB_NAMES.get(bop_id)}, not the declared ycb_name")
    T = rigid(meta["frames"]["T_asset_bop"])

    ply, png = f"models/obj_{bop_id:06d}.ply", f"models/obj_{bop_id:06d}.png"
    archive.extract(ply, workdir)
    archive.extract(png, workdir)  # the PLY names its texture; trimesh resolves it beside the PLY
    bop = trimesh.load(workdir / ply, process=False)
    bop.apply_scale(1e-3)
    ours = trimesh.load(visual_mesh(obj_dir, meta), force="mesh", process=False, skip_materials=True)

    src = trimesh.sample.sample_surface_even(bop, SAMPLES, seed=0)[0]
    dst = trimesh.sample.sample_surface_even(ours, SAMPLES, seed=1)[0]
    declared = distances_mm(src @ T[:3, :3].T + T[:3, 3], dst)
    fit = icp(src, dst, T)
    residual = fit @ np.linalg.inv(T)
    residual_deg = float(np.degrees(np.arccos(np.clip((np.trace(residual[:3, :3]) - 1) / 2, -1, 1))))
    residual_mm = float(1e3 * np.linalg.norm(residual[:3, 3]))
    p99 = float(np.percentile(declared, 99))

    bop_extent = 1e-3 * np.array([info[str(bop_id)][f"size_{a}"] for a in "xyz"])
    return {
        "name": meta["name"],
        "bop_id": bop_id,
        "ycb_name": meta["dataset"]["ycbv"]["ycb_name"],
        "source_members": {
            ply: hashlib.sha256(archive.read(ply)).hexdigest(),
            png: hashlib.sha256(archive.read(png)).hexdigest(),
        },
        "T_asset_bop": T.tolist(),
        "surface_distance_mm": {
            "mean": round(float(declared.mean()), 2),
            "p99": round(p99, 2),
            "max": round(float(declared.max()), 2),
        },
        "best_fit_residual": {"rotation_deg": round(residual_deg, 3), "translation_mm": round(residual_mm, 3)},
        # Extents along the asset frame's axes, metres.
        "asset_extent_m": [round(float(v), 4) for v in ours.extents],
        "bop_extent_in_asset_axes_m": [round(float(v), 4) for v in np.abs(T[:3, :3]) @ bop_extent],
        "pass": p99 <= MAX_P99_MM and residual_deg <= MAX_RESIDUAL_DEG and residual_mm <= MAX_RESIDUAL_MM,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--archive", type=Path, help="A local copy of the pinned ycbv_models.zip")
    parser.add_argument("--cache", type=Path, default=Path.home() / ".cache" / "prl_assets")
    parser.add_argument("--out", type=Path, default=ROOT / "docs" / "ycbv_audit.json")
    args = parser.parse_args()

    if args.archive:
        if sha256(args.archive) != ARCHIVE_SHA256:
            print(f"{args.archive} does not match the pinned SHA-256 {ARCHIVE_SHA256}", file=sys.stderr)
            return 1
        path = args.archive
    else:
        path = fetch_archive(args.cache)

    obj_dirs = sorted(meta.parent for meta in YCBV_DIR.rglob("meta.yaml"))
    with zipfile.ZipFile(path) as archive, tempfile.TemporaryDirectory() as tmp:
        info = json.loads(archive.read("models/models_info.json"))
        results = [audit(archive, Path(tmp), d, info) for d in obj_dirs]

    report = {
        "source": {"url": ARCHIVE_URL, "sha256": ARCHIVE_SHA256},
        "method": {
            "samples_per_mesh": SAMPLES,
            "distance": "symmetric nearest-neighbour surface distance, both sample sets pooled",
            "tolerances": {
                "p99_mm": MAX_P99_MM,
                "residual_rotation_deg": MAX_RESIDUAL_DEG,
                "residual_translation_mm": MAX_RESIDUAL_MM,
            },
        },
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "trimesh": trimesh.__version__,
        },
        "objects": results,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_out = tempfile.mkstemp(dir=args.out.parent, suffix=".json.part")
    with os.fdopen(fd, "w") as f:
        json.dump(report, f, indent=2)
        f.write("\n")
    os.replace(tmp_out, args.out)

    for r in results:
        d, b = r["surface_distance_mm"], r["best_fit_residual"]
        print(
            f"{'PASS' if r['pass'] else 'FAIL'} {r['name']:12s} bop {r['bop_id']:2d}  "
            f"p99 {d['p99']:.2f} mm  max {d['max']:.2f} mm  "
            f"best fit {b['rotation_deg']:.3f} deg / {b['translation_mm']:.3f} mm"
        )
    print(f"Wrote {args.out}")
    return 0 if all(r["pass"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
