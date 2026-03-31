#!/usr/bin/env python3
"""
benchmark_cli.py — colourful CLI benchmark for swarm exploration.

Measures time-to-N%-exploration for configurable robot counts, with a live
progress line that updates in-place.  Works in any terminal including VSCode.

Usage:
    scripts/benchmark_cli.py                         # defaults: 1-4 robots, 1 run, 80%
    scripts/benchmark_cli.py -r 2 4 --runs 3        # only 2 & 4 robots, 3 runs each
    scripts/benchmark_cli.py --target 90 -r 1 2      # time to 90%
    make benchmark ROBOTS="1 2" RUNS=3               # via Makefile (env-var compat)
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# ── ANSI helpers ───────────────────────────────────────────────────────────────

NO_COLOR = os.environ.get("NO_COLOR") is not None or not sys.stdout.isatty()


def _c(code: str, text: str) -> str:
    if NO_COLOR:
        return text
    return f"{code}{text}\033[0m"


def bold(t: str) -> str:
    return _c("\033[1m", t)


def dim(t: str) -> str:
    return _c("\033[2m", t)


def red(t: str) -> str:
    return _c("\033[31m", t)


def green(t: str) -> str:
    return _c("\033[32m", t)


def yellow(t: str) -> str:
    return _c("\033[33m", t)


def cyan(t: str) -> str:
    return _c("\033[36m", t)


CLEAR_LINE = "\033[2K\r" if not NO_COLOR else "\r"

# ── Progress bar ───────────────────────────────────────────────────────────────

BAR_CHARS = " ▏▎▍▌▋▊▉█"


def progress_bar(pct: float, width: int, target: float) -> str:
    """Coloured bar with a target marker."""
    w = max(width, 5)
    filled_f = pct / 100.0 * w
    filled = int(filled_f)
    frac_idx = min(int((filled_f - filled) * 8), 8)
    target_pos = min(int(target / 100.0 * w), w - 1)

    parts: list[str] = []
    for i in range(w):
        if i == target_pos:
            parts.append(yellow("▏"))
        elif i < filled:
            parts.append(green("█") if pct >= target else cyan("█"))
        elif i == filled:
            ch = BAR_CHARS[frac_idx]
            parts.append(cyan(ch) if ch != " " else dim("·"))
        else:
            parts.append(dim("·"))
    return "".join(parts)


# ── Data ───────────────────────────────────────────────────────────────────────


@dataclass
class RunResult:
    num_robots: int
    run_idx: int
    sim_s: float = 0.0
    wall_s: float = 0.0
    achieved_pct: float = 0.0
    success: bool = False
    finished: bool = False
    error: str = ""


# ── Compose runner ─────────────────────────────────────────────────────────────

WS = Path(__file__).resolve().parent.parent


def teardown() -> None:
    subprocess.run(
        [
            "docker",
            "compose",
            "-f",
            "compose.benchmark.yaml",
            "down",
            "--remove-orphans",
            "--volumes",
        ],
        cwd=WS,
        capture_output=True,
    )


def run_single(
    n_robots: int,
    run_idx: int,
    runs_total: int,
    target: float,
    sim_timeout: float,
    results_dir: Path,
) -> RunResult:
    result = RunResult(num_robots=n_robots, run_idx=run_idx)
    label = f"{n_robots} robot(s), run {run_idx}/{runs_total}"

    write_progress(f"{label}  {dim('building image …')}")

    env = {
        **os.environ,
        "NUM_ROBOTS": str(n_robots),
        "EXPLORE_TARGET": str(target),
        "SIM_TIMEOUT_S": str(sim_timeout),
    }

    proc = subprocess.Popen(
        [
            "docker",
            "compose",
            "-f",
            "compose.benchmark.yaml",
            "up",
            "--build",
            "--abort-on-container-exit",
            "--exit-code-from",
            "measurer",
            "--quiet-pull",
        ],
        cwd=WS,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    phase = "waiting"
    cur_pct = 0.0
    cur_sim = 0.0
    bar_width = min(shutil.get_terminal_size((80, 24)).columns - 50, 40)
    bar_width = max(bar_width, 10)

    write_progress(f"{label}  {yellow('waiting for frontiers …')}")

    try:
        for line in proc.stdout:
            stripped = line.strip()

            if "Timer started" in stripped and phase == "waiting":
                phase = "exploring"

            elif "BENCHMARK_PROGRESS:" in stripped:
                phase = "exploring"
                try:
                    after = stripped.split("BENCHMARK_PROGRESS:")[1].strip()
                    pct_str, rest = after.split("%", 1)
                    cur_pct = float(pct_str.strip())
                    if "sim=" in rest:
                        cur_sim = float(rest.split("sim=")[1].replace("s", "").strip())
                except (ValueError, IndexError):
                    pass

            elif "BENCHMARK_RESULT:" in stripped:
                try:
                    data = json.loads(stripped.split("BENCHMARK_RESULT:")[1].strip())
                    result.sim_s = data["elapsed_sim_s"]
                    result.wall_s = data["elapsed_wall_s"]
                    result.achieved_pct = data["achieved_pct"]
                    result.success = data["success"]
                    result.finished = True
                    cur_pct = data["achieved_pct"]
                except (ValueError, IndexError, KeyError):
                    pass
            else:
                continue

            if phase == "exploring":
                bar = progress_bar(cur_pct, bar_width, target)
                write_progress(
                    f"{label}  {bar}  {bold(f'{cur_pct:.1f}%')}"
                    f"  {dim(f'sim={cur_sim:.0f}s')}"
                )

        proc.wait()
    except KeyboardInterrupt:
        proc.terminate()
        proc.wait(timeout=10)
        raise

    # Teardown
    write_progress(f"{label}  {dim('tearing down …')}")
    teardown()

    # Collect result file
    generic = results_dir / "result.json"
    named = results_dir / f"result_{n_robots}robots_run{run_idx}.json"
    if generic.exists():
        generic.rename(named)
        if not result.finished:
            try:
                data = json.loads(named.read_text())
                result.sim_s = data["elapsed_sim_s"]
                result.wall_s = data["elapsed_wall_s"]
                result.achieved_pct = data["achieved_pct"]
                result.success = data["success"]
                result.finished = True
            except (ValueError, KeyError):
                pass

    if not result.finished:
        result.error = f"compose exit code {proc.returncode}"
        result.finished = True

    # Final status for this run
    finish_progress(label, result, target)
    return result


def write_progress(text: str) -> None:
    """Overwrite the current line with progress text."""
    sys.stdout.write(f"{CLEAR_LINE}  {text}")
    sys.stdout.flush()


def finish_progress(label: str, r: RunResult, target: float) -> None:
    """Replace the progress line with a final result line and newline."""
    if r.error:
        status = red("ERROR")
        detail = dim(r.error)
    elif r.success:
        status = green("OK")
        detail = (
            f"sim={bold(f'{r.sim_s:.1f}s')}  "
            f"wall={dim(f'{r.wall_s:.1f}s')}  "
            f"coverage={green(f'{r.achieved_pct:.1f}%')}"
        )
    else:
        status = yellow("TIMEOUT")
        detail = (
            f"sim={bold(f'{r.sim_s:.1f}s')}  "
            f"wall={dim(f'{r.wall_s:.1f}s')}  "
            f"coverage={yellow(f'{r.achieved_pct:.1f}%')}"
        )

    sys.stdout.write(f"{CLEAR_LINE}  {status}  {label}  {detail}\n")
    sys.stdout.flush()


# ── CLI ────────────────────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Benchmark swarm exploration time for different robot counts.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  %(prog)s                       # 1-4 robots, 1 run, 80%%\n"
            "  %(prog)s -r 2 4 --runs 3       # 2 & 4 robots, 3 runs\n"
            "  %(prog)s --target 90 -r 1       # single robot, 90%%\n"
            "  make benchmark ROBOTS='1 2'     # via Makefile\n"
        ),
    )
    default_robots = os.environ.get("ROBOTS", "1 2 3 4")
    default_runs = int(os.environ.get("RUNS", "1"))
    default_target = float(os.environ.get("TARGET", "80"))
    default_timeout = float(os.environ.get("SIM_TIMEOUT_S", "1800"))

    p.add_argument(
        "-r",
        "--robots",
        nargs="+",
        type=int,
        default=[int(x) for x in default_robots.split()],
        help="robot counts to test (default: 1 2 3 4)",
    )
    p.add_argument(
        "-n",
        "--runs",
        type=int,
        default=default_runs,
        help="repetitions per robot count (default: 1)",
    )
    p.add_argument(
        "-t",
        "--target",
        type=float,
        default=default_target,
        help="exploration %% target (default: 80)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=default_timeout,
        help="sim-time timeout per run in seconds (default: 1800)",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    robots = sorted(args.robots)
    results_dir = WS / "benchmark_results"
    results_dir.mkdir(exist_ok=True)

    # Ctrl+C handler
    def handle_sigint(*_: object) -> None:
        sys.stdout.write("\n")
        sys.stdout.flush()
        print(yellow("\nInterrupted — tearing down containers …"))
        teardown()
        sys.exit(1)

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    # Header
    cols = shutil.get_terminal_size((80, 24)).columns
    print()
    print(f"  {cyan(bold('Swarm Exploration Benchmark'))}")
    print(f"  {dim('━' * min(cols - 4, 52))}")
    robot_str = ", ".join(str(r) for r in robots)
    print(
        f"  Target: {bold(f'{args.target:.0f}%')}  "
        f"Robots: {bold(f'[{robot_str}]')}  "
        f"Runs: {bold(str(args.runs))}  "
        f"Timeout: {dim(f'{args.timeout:.0f}s')}"
    )
    print()

    results: list[RunResult] = []
    t0 = time.monotonic()

    for n in robots:
        for run_idx in range(1, args.runs + 1):
            r = run_single(
                n, run_idx, args.runs, args.target, args.timeout, results_dir
            )
            results.append(r)

    elapsed_total = time.monotonic() - t0

    # Summary
    print()
    print(f"  {cyan(bold('Results'))}  {dim(f'({elapsed_total:.0f}s total)')}")
    print(f"  {dim('━' * min(cols - 4, 52))}")
    print(f"  {bold(f'{'Robots':<8}  {'Sim (s)':>14}  {'Wall (s)':>14}')}")
    print(f"  {dim(f'{'─' * 8}  {'─' * 14}  {'─' * 14}')}")

    for n in robots:
        ok = [r for r in results if r.num_robots == n and r.success]
        if not ok:
            failed = [r for r in results if r.num_robots == n]
            if failed:
                print(f"  {n:<8}  {yellow('TIMEOUT'):>14}  {yellow('TIMEOUT'):>14}")
            else:
                print(f"  {n:<8}  {red('FAILED'):>14}  {red('FAILED'):>14}")
            continue
        sim_vals = [r.sim_s for r in ok]
        wall_vals = [r.wall_s for r in ok]

        def fmt(vals: list[float]) -> str:
            m = statistics.mean(vals)
            if len(vals) == 1:
                return f"{m:.1f}"
            sd = statistics.stdev(vals)
            return f"{m:.1f} +/-{sd:.1f}"

        print(
            f"  {green(str(n)):<17}  {bold(fmt(sim_vals)):>23}"
            f"  {dim(fmt(wall_vals)):>23}"
        )

    print()
    print(f"  {dim(f'Results saved to {results_dir}/')}")
    print()

    # Plain-text summary
    summary_path = results_dir / "summary.txt"
    with open(summary_path, "w") as f:
        f.write(f"Benchmark: time to {args.target:.0f}% exploration\n\n")
        f.write(f"{'Robots':<8}  {'Sim (s)':>14}  {'Wall (s)':>14}\n")
        f.write(f"{'─' * 8}  {'─' * 14}  {'─' * 14}\n")
        for n in robots:
            ok = [r for r in results if r.num_robots == n and r.success]
            if not ok:
                f.write(f"{n:<8}  {'FAILED':>14}  {'FAILED':>14}\n")
                continue
            sim_m = statistics.mean(r.sim_s for r in ok)
            wall_m = statistics.mean(r.wall_s for r in ok)
            f.write(f"{n:<8}  {sim_m:>14.1f}  {wall_m:>14.1f}\n")
        f.write(f"\nTotal benchmark time: {elapsed_total:.0f}s\n")


if __name__ == "__main__":
    main()
