#!/usr/bin/python3

import sys
import os
import csv
import time
from pathlib import Path
import argparse
import subprocess

def run_test(pathin, pathout, fps, start, stop):
    print("Starting test...", flush=True)
    print("Running stopwatch...", flush=True)
    stopwatch = subprocess.Popen(['devel/lib/stopwatch/stopwatch'], stdout=None, stderr=None)
    print("Running policy...", flush=True)
    start = subprocess.Popen(start.split(' '))
    start.wait()
    print("Running tester...", flush=True)
    tester = subprocess.Popen(['devel/lib/tracking_tester/tracking_tester', '-i',
            str(pathin), '-o', str(pathout), '-f', str(fps)])
    tester.wait()
    print("Stopping policy...", flush=True)
    stop = subprocess.Popen(stop.split(' '))
    stop.wait()
    print("Killing stopwatch...", flush=True)
    subprocess.call(['rosnode', 'kill', 'stopwatch'])
    stopwatch.wait()
    print("Test done", flush=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('in_paths', nargs='+', type=Path)
    parser.add_argument('out_path', type=Path)
    parser.add_argument('--passes', type=int, default=1, dest='passes')
    parser.add_argument('--config_path', type=Path, default='test.config', dest='config_path')
    args = parser.parse_args()

    iteration = 0
    with open(args.config_path, 'r') as configfile:
        config_lines = configfile.read().splitlines()
        if len(config_lines) % 4:
            raise RuntimeError("Invalid config file - number of lines not divisible by 4")
        configs = []
        for i in range(len(config_lines) // 4):
            config = {}
            config["name"] = config_lines[i * 4]
            config["fps"] = int(config_lines[i * 4 + 1])
            config["start"] = config_lines[i * 4 + 2]
            config["stop"] = config_lines[i * 4 + 3]
            configs.append(config)
        with open(args.out_path / 'test.index', 'w') as index:
            index.write(f"passes={args.passes}\n")
            index.write(f"tests={len(args.in_paths)}\n")
            for counter, path in enumerate(args.in_paths):
                index.write(f"test{counter + 1}={path.name}\n")
            index.write(f"policies={len(configs)}\n")
            for counter, config in enumerate(configs):
                index.write(f"policyname{counter + 1}={config['name']}\n")
                index.write(f"policyfps{counter + 1}={config['fps']}\n")

        for config in configs:
            policy_name = config['name']
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
                    run_test(in_path, pathout, config['fps'], config['start'],
                            config['stop'])
                    time.sleep(1)

if __name__ == "__main__":
    main() 
