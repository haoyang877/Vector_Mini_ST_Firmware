"""Check Keil source paths, include directories and local firmware includes.

Run from any directory: python tools/build/check_project_layout.py
This only reads files; it does not build, flash or access motor hardware.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS

from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

def resolve_path(base, value):
    return (base / value.replace('\\', '/')).resolve()


def check_project(project):
    errors = []
    tree = ET.parse(project)
    include_dirs = []
    sources = []
    for node in tree.findall('.//IncludePath'):
        for value in (node.text or '').split(';'):
            if not value:
                continue
            path = resolve_path(project.parent, value)
            if re.match(r'^[A-Za-z]:', value) or value.startswith(('/', '\\')):
                errors.append(f'non-portable include directory: {value}')
            if not path.is_dir():
                errors.append(f'missing include directory: {value}')
            if path not in include_dirs:
                include_dirs.append(path)
    for node in tree.findall('.//FilePath'):
        value = node.text or ''
        path = resolve_path(project.parent, value)
        if re.match(r'^[A-Za-z]:', value) or value.startswith(('/', '\\')):
            errors.append(f'non-portable source path: {value}')
        if not path.is_file():
            errors.append(f'missing source: {value}')
        if path in sources:
            errors.append(f'duplicate source: {value}')
        sources.append(path)

    # Include headers as well as project-listed C files: Keil does not list every
    # header, and a moved header can contain another relative include.
    owned = ('firmware/app', 'firmware/common', 'firmware/motor', 'firmware/services', 'firmware/communication', 'firmware/platform/api', 'firmware/platform/stm32g4/bsp', 'firmware/platform/stm32g4/ports', 'tests/hil/firmware', 'firmware/platform/stm32g4/cubemx/Core', 'firmware/third_party/segger_rtt')
    for folder in owned:
        for source in (ROOT / folder).rglob('*'):
            if source.suffix not in ('.c', '.h'):
                continue
            # CubeMX lists optional HAL modules behind preprocessor guards.
            # Their enabled dependencies are verified by the real Keil build.
            if source.name == 'stm32g4xx_hal_conf.h':
                continue
            text = source.read_text(encoding='utf-8-sig', errors='surrogateescape')
            for value in re.findall(r'^\s*#\s*include\s+"([^"]+)"', text, re.M):
                if not any(resolve_path(base, value).is_file()
                           for base in [source.parent, *include_dirs]):
                    errors.append(f'{source.relative_to(ROOT)}: unresolved include "{value}"')
    return errors, len(sources), len(include_dirs)


def main():
    failed = False
    for name in ('Vector_Mini_ST', 'Vector_Mini_ST_HIL'):
        project = ROOT / 'firmware/platform/stm32g4/cubemx/MDK-ARM' / f'{name}.uvprojx'
        errors, sources, includes = check_project(project)
        for error in errors:
            print(f'{name}: {error}', file=sys.stderr)
        print(f'{name}: {sources} files, {includes} include directories, {len(errors)} errors')
        failed |= bool(errors)
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
