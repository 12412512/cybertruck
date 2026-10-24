"""把正式纯C观察器构建为主机模型库；不编译/打开任何硬件驱动。"""

import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    control = ROOT / 'firmware/User/Control'
    device = ROOT / 'firmware/User/Device'
    comm = ROOT / 'firmware/User/Comm'
    parser = argparse.ArgumentParser(description='仅构建主机纯计算模型')
    parser.add_argument('--output', type=Path, default=ROOT / 'output/native' /
                        ('odometry.dll' if os.name == 'nt' else 'odometry.so'))
    args = parser.parse_args()
    library = args.output.resolve()
    library.parent.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, '-m', 'ziglang', 'cc', '-shared', '-std=c99', '-O2',
               '-Wall', '-Wextra', '-Werror', '-DODOM_HOST', '-I', str(control),
               '-I', str(device), '-I', str(comm),
               *(str(control / name) for name in ('odom.c', 'kin.c', 'yaw.c', 'curve.c',
                                                  'line.c', 'path.c', 'ref.c', 'track.c')),
               str(device / 'stamp.c'), str(comm / 'feedback.c'), '-o', str(library)]
    if os.name != 'nt':
        command += ['-fPIC', '-lm']
    subprocess.run(command, check=True)
    print(library)


if __name__ == '__main__':
    main()
