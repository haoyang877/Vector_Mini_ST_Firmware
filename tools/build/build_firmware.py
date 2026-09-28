"""Rebuild the APP Keil targets. This command never downloads firmware."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from project_paths import ROOT, KEIL


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uv4', default=shutil.which('UV4.exe') or 'C:/Keil_v5/UV4/UV4.exe')
    parser.add_argument('--target', choices=('normal', 'hil', 'all'), default='all')
    args = parser.parse_args()
    if not Path(args.uv4).is_file():
        parser.error('Keil executable not found; specify --uv4 /path/to/UV4.exe')
    log_dir = ROOT / 'outputs/build/logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    targets = {'normal': 'Vector_Mini_ST', 'hil': 'Vector_Mini_ST_HIL'}
    for kind, name in targets.items():
        if args.target not in (kind, 'all'):
            continue
        log = log_dir / (name + '.log')
        result = subprocess.run([args.uv4, '-r', str(KEIL / (name + '.uvprojx')),
                                 '-j0', '-o', str(log)], cwd=KEIL)
        text = log.read_text(errors='replace') if log.exists() else ''
        print('\n'.join(text.splitlines()[-6:]))
        if result.returncode > 1 or '0 Error(s)' not in text:
            print(f'Build failed. See {log}', file=sys.stderr)
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
