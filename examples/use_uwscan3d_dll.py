from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(r"C:\Users\AAA\Desktop\UWScan2")
sys.path.insert(0, str(ROOT / "python"))

from uwscan3d import UWScan3DDll  # noqa: E402


def main() -> None:
    dll = UWScan3DDll.discover(ROOT)
    print("DLL:", dll.path)
    print("Version:", dll.version())
    print("Default Basler config:", dll.default_basler_config())


if __name__ == "__main__":
    main()
