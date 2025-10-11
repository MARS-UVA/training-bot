#!/usr/bin/env python3

import subprocess
import sys
from pathlib import Path


def install_package(package_path: Path) -> int:
    proc = subprocess.Popen([sys.executable, '-m', 'pip', 'install', package_path, '--break-system-packages'],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            universal_newlines=True)

    if proc.stdout is None:
        raise RuntimeError('stdout was not piped')
    for line in iter(proc.stdout.readline, ''):
        print(f'[{package_path.name}]--> {line}', end='')

    return proc.wait()



def main() -> None:
    for package in Path('src').glob('*'):
        if (package / 'setup.py').exists():
            print(f'Installing {package.name}...')
            if exit_code := install_package(package_path=package):
                print(f'Failed to install {package.name} with exit code {exit_code} (see above output)', file=sys.stderr)
                sys.exit(exit_code)
            print(f'Successfully installed {package.name}')
    print('All Python packages successfully installed')


if __name__ == '__main__':
    main()
