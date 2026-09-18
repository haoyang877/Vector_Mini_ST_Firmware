"""STM32G431 bench-only fallback after the normal HIL STOP has failed.

This is not a firmware safety watchdog. It requires a working SWD connection.
Leave the CPU halted only after phase output disables have been read back;
resuming requires explicit recovery, never an automatic restart of the trial.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def disable_outputs(j):
    ccer, bdtr = 0x40012C20, 0x40012C44
    j.memory_write32(bdtr, [j.memory_read32(bdtr, 1)[0] & ~0x8000])
    j.memory_write32(ccer, [j.memory_read32(ccer, 1)[0] & ~0x555])
    require(j.memory_read32(ccer, 1)[0] & 0x555 == 0, 'phase outputs not disabled')
    require(j.memory_read32(bdtr, 1)[0] & 0x8000 == 0, 'main output not disabled')
    j.halt()
    require(j.halted(), 'CPU halt after output disable failed')
    # Defend against firmware re-enabling in the interval preceding the halt.
    j.memory_write32(bdtr, [j.memory_read32(bdtr, 1)[0] & ~0x8000])
    j.memory_write32(ccer, [j.memory_read32(ccer, 1)[0] & ~0x555])
    state = dict(ccer=j.memory_read32(ccer, 1)[0], bdtr=j.memory_read32(bdtr, 1)[0],
                 cpu_halted=j.halted())
    require(state['ccer'] & 0x555 == 0 and state['bdtr'] & 0x8000 == 0,
            'phase outputs changed state after CPU halt')
    return state
