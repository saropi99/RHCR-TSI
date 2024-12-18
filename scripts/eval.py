#!/usr/bin/env python3
import argparse
import datetime
import os
import signal
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any, Dict, List, Optional

import atomics
import git
import pandas as pd
import yaml

RHCR_BINARY = Path.cwd() / "lifelong"
MAPS_DIR = Path("maps")


@dataclass
class RHCRResult:
    config: Dict[str, Any]
    return_code: int
    completed_tasks: Optional[int]
    runtime_s: Optional[float]


def run_instance(
    timeout_s: int,
    instance: Dict[str, Any],
    total_instances: int,
    execution_counter: atomics.INTEGRAL,
) -> RHCRResult:
    proc = None
    try:
        with TemporaryDirectory() as tmpdir:
            command = ["timeout", timeout_s, RHCR_BINARY, f"--output={tmpdir}"] + [
                f"--{key}={value}" for key, value in instance.items()
            ]
            command = [str(c) for c in command]
            print(
                datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%S ")
                + f"Running command ({execution_counter.fetch_inc() + 1}/{total_instances}): {' '.join(command)}"
            )
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
                return RHCRResult(instance, rv, num_completed, runtime_s)
            return RHCRResult(instance, rv, None, None)
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


def main(*, config_path: Path, dry_run: bool) -> None:
    config = load_config(config_path)

    instances: List[Dict[str, Any]] = []

    for solver_name, solver_info in config["solvers"].items():
        for map in config["maps"]:
            for simulation_window in config["simulation_windows"]:
                for num_agents in solver_info["num_agents"]:
                    for seed in range(config["n_seeds"]):
                        instances.append(
                            {
                                "scenario": "MT",
                                "solver": solver_name,
                                "map": MAPS_DIR / map,
                                "agentNum": num_agents,
                                "simulation_window": simulation_window,
                                "planning_window": simulation_window * 2,
                                "seed": seed,
                                "simulation_time": config["simulation_time"],
                            }
                        )

    max_workers = config.get("max_workers", 1)
    print(f"Running {len(instances)} instances across {max_workers} workers")
    if dry_run:
        return
    execution_counter = atomics.atomic(width=4, atype=atomics.INT)
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = [
            executor.submit(
                run_instance,
                config["time_limit_sec"],
                instance,
                len(instances),
                execution_counter,
            )
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
    parser.add_argument(
        "--dry-run",
        action="store_true",
        default=False,
        help="Preview number of instances and workers",
    )
    start_time = time.perf_counter()
    main(**vars(parser.parse_args()))
    print(f"Total experiment runtime: {time.perf_counter() - start_time:.2f}s")
