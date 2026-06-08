#!/usr/bin/env python3
import glob
import os
from datetime import datetime


def make_run_dir(results_dir, stamp=None):
    """Create one timestamped run directory under results_dir."""
    base_dir = os.path.abspath(os.path.expanduser(results_dir))
    os.makedirs(base_dir, exist_ok=True)

    base_stamp = stamp or datetime.now().strftime("%Y%m%d_%H%M%S")
    run_stamp = base_stamp

    while True:
        run_dir = os.path.join(base_dir, run_stamp)
        try:
            os.makedirs(run_dir)
            return run_dir, run_stamp
        except FileExistsError:
            suffix = 1
            while os.path.exists(os.path.join(base_dir, "%s_%02d" % (base_stamp, suffix))):
                suffix += 1
            run_stamp = "%s_%02d" % (base_stamp, suffix)


def matching_files(results_dir, pattern):
    """Return files matching pattern in results_dir or any timestamped subdir."""
    base_dir = os.path.abspath(os.path.expanduser(results_dir))
    seen = set()
    files = []

    for search_pattern in (
        os.path.join(base_dir, pattern),
        os.path.join(base_dir, "**", pattern),
    ):
        for path in glob.glob(search_pattern, recursive=True):
            if os.path.isfile(path) and path not in seen:
                seen.add(path)
                files.append(path)

    return files


def latest_matching_file(results_dir, patterns):
    if isinstance(patterns, str):
        patterns = [patterns]

    files = []
    for pattern in patterns:
        files.extend(matching_files(results_dir, pattern))

    if not files:
        raise FileNotFoundError("No result file found in %s" % results_dir)

    return max(files, key=os.path.getmtime)
