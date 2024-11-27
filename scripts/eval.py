#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

RHCR_BINARY = Path.cwd() / "lifelong"
MAPS_DIR = Path("maps")


def run_instance(timeout_s: int, instance: Dict[str, Any]) -> Optional[str]:
    proc = None
    try:
        command = ["timeout", timeout_s, RHCR_BINARY] + [
            f"--{key}={value}" for key, value in instance.items()
        ]
        command = [str(c) for c in command]
        print(f"Running command: {' '.join(command)}")
        # proc = subprocess.Popen(
        #     command,
        #     preexec_fn=os.setsid,
        #     stdout=subprocess.PIPE,
        #     stderr=subprocess.PIPE,
        # )
        # return proc.wait()
    except subprocess.CalledProcessError as e:
        print(f"Instance failed with error:\n{e.stderr}")
        return None
    finally:
        if proc and proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


def load_config(config_path: Path) -> dict:
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def main(*, config_path: Path) -> None:
    config = load_config(config_path)

    instances: List[Dict[str, Any]] = []
    for scenario in config["scenarios"]:
        for solver in config["solvers"]:
            for map in config["maps"]:
                for num_agents in config["num_agents"]:
                    for simulation_window in config["simulation_windows"]:
                        for seed in config["seeds"]:
                            instances.append(
                                {
                                    "scenario": scenario,
                                    "solver": solver,
                                    "map": MAPS_DIR / map,
                                    "agentNum": num_agents,
                                    "simulation_window": simulation_window,
                                    "seed": seed,
                                    "simulation_time": config["simulation_time"],
                                }
                            )

    max_workers = config.get("max_workers", 1)
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = [
            executor.submit(run_instance, config["time_limit_sec"], instance)
            for instance in instances
        ]
        results = []
        for future in futures:
            result = future.result()
            if result:
                results.append(result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config_path", type=str, help="Path to config file")
    main(**vars(parser.parse_args()))
