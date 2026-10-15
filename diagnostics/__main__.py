"""__main__"""

import importlib
import sys

REPORTS = {
    "sticky-sources": "sticky_source_check",
    "sticky-history": "sticky_two_buckets",
    "sticky-geometry": "sticky_cells_describe",
    "inflate-unknown": "check_inflate_unknown",
    "snapshot-diff": "diff_snapshots",
    "last-mark": "last_mark_attribution",
    "residue": "residue_analyze",
}


def main():
    name = sys.argv.pop(1) if len(sys.argv) > 1 else ""
    if name not in REPORTS:
        print("Reports: " + ", ".join(REPORTS), file=sys.stderr)
        return 2
    try:
        return importlib.import_module("diagnostics." + REPORTS[name]).main()
    except (ValueError, OSError) as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
