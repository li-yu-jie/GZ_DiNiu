#!/usr/bin/env python3
"""兼容入口：转到 src/io_control_py/io_control_node.py。"""

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'io_control_py'
sys.path.insert(0, str(PACKAGE_ROOT))

from io_control_py.io_control_node import main  # noqa: E402


if __name__ == '__main__':
    main()
