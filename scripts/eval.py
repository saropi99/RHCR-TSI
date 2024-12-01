#!/usr/bin/env python3
import argparse
import datetime
import os
import signal
import subprocess
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any, Dict, List, Optional

import git
import pandas as pd
import yaml

RHCR_BINARY = Path.cwd() / "lifelong"
MAPS_DIR = Path("maps")


@dataclass
class RHCRResult:
    config: Dict[str, Any]
    success: bool
    completed_tasks: Optional[int]
    runtime_s: Optional[float]


def run_instance(timeout_s: int, instance: Dict[str, Any]) -> RHCRResult:
    proc = None
    try:
        with TemporaryDirectory() as tmpdir:
            command = ["timeout", timeout_s, RHCR_BINARY, f"--output={tmpdir}"] + [
                f"--{key}={value}" for key, value in instance.items()
            ]
            command = [str(c) for c in command]
            print(f"Running command: {' '.join(command)}")
            start_time = time.perf_counter()
            proc = subprocess.Popen(
                command,
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            rv = proc.wait()
            runtime_s = time.perf_counter() - start_time
            if rv == 0:
                with open(Path(tmpdir) / "tasks.txt", "r") as f:
                    num_completed = int(f.readlines()[-1].split(" ")[-1].strip())
                return RHCRResult(instance, True, num_completed, runtime_s)
            return RHCRResult(instance, False, None, None)
    except subprocess.CalledProcessError as e:
        print(f"Instance failed with error:\n{e.stderr}")
        return None
    finally:
        if proc and proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


def load_config(config_path: Path) -> dict:
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def write_config(config: dict, config_path: Path) -> None:
    with open(config_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False)


def main(*, config_path: Path) -> None:
    config = load_config(config_path)

    instances: List[Dict[str, Any]] = []
    for scenario in config["scenarios"]:
        for solver in config["solvers"]:
            for map in config["maps"]:
                for num_agents in config["num_agents"]:
                    for simulation_window in config["simulation_windows"]:
                        for planning_window in config["planning_windows"]:
                            for seed in config["seeds"]:
                                instances.append(
                                    {
                                        "scenario": scenario,
                                        "solver": solver,
                                        "map": MAPS_DIR / map,
                                        "agentNum": num_agents,
                                        "simulation_window": simulation_window,
                                        "planning_window": planning_window,
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
        results: List[RHCRResult] = []
        for future in futures:
            result = future.result()
            if result:
                results.append(result)

    df = pd.DataFrame(
        [
            {
                **result.config,
                "completed_tasks": result.completed_tasks,
                "runtime_s": result.runtime_s,
            }
            for result in results
        ]
    )
    result_dir = Path(config["root"]) / datetime.datetime.now().strftime(
        "%Y-%m-%dT%H-%M-%S"
    )
    result_dir.mkdir(exist_ok=True, parents=True)

    new_config_fname = result_dir / "config.yaml"
    repo = git.Repo(search_parent_directories=True)
    config["git_dirty"] = repo.is_dirty()
    config["git_branch"] = repo.active_branch.name
    config["git_hash"] = repo.active_branch.commit.hexsha
    write_config(config, new_config_fname)
    print(f"Config saved to {new_config_fname.resolve()}")

    results_fname = result_dir / "result.csv"
    df.to_csv(results_fname, index=False)
    print(f"Results saved to {results_fname.resolve()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config_path", type=str, help="Path to config file")
    main(**vars(parser.parse_args()))
