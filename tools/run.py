"""Run a repository tool by name: python tools/run.py --list."""
from pathlib import Path
import runpy
import sys

from project_paths import ROOT


def commands():
    result = {}
    for directory in ('tools/build', 'tools/flash', 'tools/bench', 'tools/analysis',
                      'tests/unit/native'):
        for path in sorted((ROOT / directory).glob('*.py')):
            if path.name.startswith('_'):
                continue
            if path.stem in result:
                raise ValueError(f'Duplicate command: {path.stem}')
            result[path.stem] = path
    return result


def main():
    available = commands()
    if len(sys.argv) == 1 or sys.argv[1] == '--list':
        for name, path in available.items():
            print(f'{name:36} {path.relative_to(ROOT).as_posix()}')
        return 0
    name = Path(sys.argv[1]).stem
    if name not in available:
        print(f'Unknown command: {name}. Use --list.', file=sys.stderr)
        return 2
    path = available[name]
    sys.argv = [str(path), *sys.argv[2:]]
    runpy.run_path(str(path), run_name='__main__')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
