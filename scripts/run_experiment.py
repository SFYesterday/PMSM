"""通用实验入口，便于后续按论文切换复现模块。"""

from __future__ import annotations

import argparse
from importlib import import_module
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if SRC.exists() and str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

DEFAULT_MODULE = "pmsm.papers.basic_fcs_mpc"


def main() -> None:
    parser = argparse.ArgumentParser(description="运行 PMSM 论文复现实验")
    parser.add_argument(
        "--module",
        default=DEFAULT_MODULE,
        help="要运行的模块路径，例如 pmsm.papers.basic_fcs_mpc",
    )
    args = parser.parse_args()
    module = import_module(args.module)
    module.main()


if __name__ == "__main__":
    main()
