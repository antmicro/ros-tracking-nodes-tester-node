#!/usr/bin/python3

import sys
import os
import csv
import time
from pathlib import Path
import argparse

def run_test(pathin, pathout, config):
    os.system(f"./ci-run-tester.sh {config} {pathin} {pathout}")
    time.sleep(0.5)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('in_paths', nargs='+', type=Path)
    parser.add_argument('out_path', type=Path)
    parser.add_argument('--passes', type=int, default=1, dest='passes')
    args = parser.parse_args()

    iteration = 0
    with open('test.config', 'r') as configfile:
        configs = configfile.read().splitlines()
        with open(args.out_path / 'test.index', 'w') as index:
            index.write(f"passes={args.passes}\n")
            index.write(f"tests={len(args.in_paths)}\n")
            for counter, path in enumerate(args.in_paths):
                index.write(f"test{counter + 1}={path.name}\n")
            index.write(f"policies={len(configs)}\n")
            for counter, config in enumerate(configs):
                fps, name = config.split(' ')
                index.write(f"policyname{counter + 1}={name}\n")
                index.write(f"policyfps{counter + 1}={fps}\n")

        for config in configs:
            policy_name = config.split(' ')[1]
            path = Path(args.out_path / policy_name)
            try:
                if not path.exists():
                    path.mkdir()
            except FileNotFoundError:
                print(f'Parent for {path} does not exist')
                return 1
            for counter in range(args.passes):
                for in_path in args.in_paths:
                    iteration += 1
                    info = (f'[{iteration}/{args.passes * len(configs) * len(args.in_paths)}]'
                            f' Policy: {policy_name}, Pass: {counter + 1}, '
                            f'Test: {in_path.name}')
                    print(len(info) * '-')
                    print(info, flush=True)
                    pathout = path / \
                        Path(f"{in_path.name}_{str(counter + 1)}").with_suffix('.csv')
                    run_test(in_path, pathout, config)

if __name__ == "__main__":
    main() 