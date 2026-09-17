"""Repository paths shared by direct CLI tools and offline test runners."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / 'firmware'
CUBEMX = FIRMWARE / 'platform/stm32g4/cubemx'
KEIL = CUBEMX / 'MDK-ARM'
BUILD = ROOT / 'outputs/build/keil'

# Existing tools remain directly executable without an editable installation.
for relative in ('tools/bench', 'tools/analysis', 'tools/flash', 'tools/build',
                 'tests/unit/native'):
    directory = str(ROOT / relative)
    if directory not in sys.path:
        sys.path.append(directory)

NATIVE_INCLUDE_DIRS = (
    'firmware/app', 'firmware/common', 'firmware/motor',
    'firmware/motor/foc', 'firmware/motor/position', 'firmware/motor/trajectory',
    'firmware/motor/identification', 'firmware/motor/protection',
    'firmware/services/parameters', 'firmware/services/telemetry',
    'firmware/communication/can', 'firmware/communication/protocol',
    'firmware/platform/api', 'firmware/platform/stm32g4/bsp',
    'firmware/third_party/segger_rtt', 'tests/hil/firmware',
)
NATIVE_INCLUDE_FLAGS = [item for directory in NATIVE_INCLUDE_DIRS
                        for item in ('-I', str(ROOT / directory))]
