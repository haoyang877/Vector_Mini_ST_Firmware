"""独立发布包 GUI 回归用内存探针，在测试进程内运行包中的维护脚本。"""

import sys
from pathlib import Path
from types import SimpleNamespace

KIT = Path(sys.executable).resolve().parents[2]
sys.path.insert(0, str(KIT / "loader/tools"))
import device_maintenance  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "unit"))
from test_device_maintenance import MaintenanceTests, Probe  # noqa: E402


class GuiProbe(Probe):
    def __init__(self):
        super().__init__()
        self.flash[0x1C000:0x20000] = MaintenanceTests().parameter_image()

    def connected_emulators(self):
        return [SimpleNamespace(SerialNumber=314159)]

    def open(self, serial_no):
        assert serial_no == 314159

    def connect(self, device, speed):
        assert device == "AT32F456CCU7" and speed == 1000

    def set_tif(self, _interface):
        pass

    def exec_command(self, _command):
        pass

    def close(self):
        if sys.argv[1] == "backup":
            assert not self.writes
        else:
            assert self.writes == [(0x08004000, 98304), (0x08000000, 16384), (0x08004000, 2048)]


if __name__ == "__main__":
    assert Path(device_maintenance.__file__).resolve().is_relative_to(KIT)
    device_maintenance.create_probe = lambda _dll: GuiProbe()
    device_maintenance.main()
