"""编译离线 C 示例，并与独立 Python 编码器逐字节对照；不访问硬件。"""

from __future__ import annotations

import os
import runpy
import struct
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from harness.common import zig_path
from project_paths import ROOT


class CompanyCanFdExampleTests(unittest.TestCase):
    """跨实现黄金向量检查与 C 内置拒收测试。"""

    def test_native_example_against_reference(self) -> None:
        """检查编译、执行、位置目标和含负速度反馈的完整帧。"""
        compiler = zig_path()
        if compiler is None:
            self.skipTest("安装 dev 依赖或设置 HARNESS_CC 后运行原生示例。")
        out = ROOT / "outputs/tests/company_canfd_example"
        out.mkdir(parents=True, exist_ok=True)
        executable = out / (
            "company_canfd_position.exe" if os.name == "nt" else "company_canfd_position"
        )
        command = [str(compiler)]
        if "zig" in compiler.name.lower():
            command.append("cc")
        command += [
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            str(ROOT / "host_app/examples/company_canfd_position.c"),
            "-o",
            str(executable),
        ]
        built = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
        (out / "build.log").write_text(built.stdout + built.stderr, encoding="utf-8")
        self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
        run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=15)
        (out / "run.log").write_text(run.stdout + run.stderr, encoding="utf-8")
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        reference = runpy.run_path(str(ROOT / "tools/analysis/protocol_vectors_reference.py"))
        cases = (
            ("CONTROL", 105, 2, struct.pack("<BBHi", 105, 2, 0x1234, 1000), 2, 3, 2),
            ("FEEDBACK", 124, 10, struct.pack("<ihh", 950, -1200, 300), 3, 2, 3),
        )
        for label, msgtype, sequence, payload, source, destination, priority in cases:
            with self.subTest(label=label):
                record = reference["fd_frames"](
                    msgtype,
                    sequence,
                    payload,
                    flags=0,
                    src=source,
                    dst=destination,
                    priority=priority,
                )[0]
                expected = (
                    f"{label} ID={record['can_id'][2:]} IDE=1 FDF=1 BRS=1 DLC=13 LEN=32 "
                    f"DATA={record['data']}"
                )
                self.assertIn(expected, run.stdout.splitlines())
        self.assertIn("PASS offline codec checks; no hardware accessed", run.stdout)


if __name__ == "__main__":
    unittest.main()
