#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""Check prl_assets objects the way CI does.

Each object's compiled MuJoCo model is checked by ``prl_assets.authoring.check``: a free
joint, collision and visual groups, compiled mass equal to the declared mass, collision
geometry that matches the visual mesh, a model file pose that rests on z = 0, and, for
objects declared open, an interior that a dropped object reaches. docs/schema.md gives
the tolerances.

Usage:
    uv run --extra authoring python scripts/validate_objects.py              # every object
    uv run --extra authoring python scripts/validate_objects.py plastic_bowl # some objects

Exits 1 if any object fails, 2 for an unknown object name.
"""

import argparse
import sys

from prl_assets import authoring


def main() -> int:
    parser = argparse.ArgumentParser(description="Check prl_assets objects the way CI does")
    parser.add_argument("objects", nargs="*", help="Object names (default: all)")
    args = parser.parse_args()

    known = authoring.object_dirs()
    names = args.objects or sorted(known)
    unknown = [n for n in names if n not in known]
    if unknown:
        print(f"Unknown objects: {', '.join(unknown)}. Available: {', '.join(sorted(known))}")
        return 2

    reports = [authoring.check(n) for n in names]
    for report in reports:
        print(report, end="\n\n")
    failed = [r.name for r in reports if not r.ok]
    print(
        f"{len(reports) - len(failed)}/{len(reports)} objects pass"
        + (f"; failing: {', '.join(failed)}" if failed else "")
    )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
