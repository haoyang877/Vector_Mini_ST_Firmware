"""已有离线验证入口的Loader适配，不触发硬件。"""

from pathlib import Path
import runpy

if __name__ == "__main__":
    runpy.run_path(
        str(Path(__file__).resolve().parents[2] / "loader/run_tests.py"),
        run_name="__main__",
    )
