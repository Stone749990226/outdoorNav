"""Offline generation entrypoint executed by the configured data Python."""

import argparse
from pathlib import Path

from .bundle import ROOT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=["survey", "scene"])
    parser.add_argument("--loop", type=Path, default=ROOT / "1.ubx")
    parser.add_argument("--branch", type=Path, default=ROOT / "2.ubx")
    args = parser.parse_args()
    if args.command == "survey":
        from .survey import build

        build(
            args.loop, args.branch, origin_path=ROOT / "scenes/north_campus/origin.json"
        )
        print(
            "Survey regenerated. Run ./nav scene build to publish a matching scene bundle."
        )
    else:
        from .scene import build

        build(ROOT / "scenes/north_campus/generated")


if __name__ == "__main__":
    main()
