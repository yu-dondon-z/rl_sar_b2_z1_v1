#!/usr/bin/env python3
"""Explicit READ-ONLY native probe exit regression; never starts a controller."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cycles", type=int, default=10)
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.cycles <= 100:
        parser.error("cycles must be 1..100")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    binary, model = str(args.binary.resolve()), str(args.model.resolve())
    results = []

    def run(name, seconds, expected=0, interrupt=None, interface=None):
        prefix = (args.output_dir / name).resolve()
        env = dict(os.environ)
        if args.sanitize:
            env["ASAN_OPTIONS"] = "halt_on_error=1:detect_leaks=1:symbolize=0"
            env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
        command = [binary, "--subscribe", model, interface or args.interface,
                   str(prefix), "--seconds", str(seconds)]
        started = time.monotonic()
        with prefix.with_suffix(".stdout.log").open("w") as stdout, \
                prefix.with_suffix(".stderr.log").open("w") as stderr:
            process = subprocess.Popen(command, stdout=stdout, stderr=stderr, env=env)
            try:
                if interrupt is not None:
                    # Wait for an initialized subscriber, not a fixed startup
                    # delay that may signal ASan/loader initialization instead.
                    deadline = time.monotonic() + 15
                    while "[Read-only transport]" not in prefix.with_suffix(".stdout.log").read_text():
                        if process.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError("No transport startup marker before signal")
                        time.sleep(.05)
                    time.sleep(1)
                    process.send_signal(interrupt)  # only this diagnostic child
                code = process.wait(timeout=seconds + 20)
            except subprocess.TimeoutExpired:
                code = -999  # retained as a failed case, never a successful capture
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
        error = prefix.with_suffix(".stderr.log").read_text()
        result = {"name": name, "exit": code, "expected_exit": expected,
                  "wall_seconds": time.monotonic() - started}
        if code == expected and expected in (0, 128 + signal.SIGINT, 128 + signal.SIGTERM):
            report = json.loads(prefix.with_suffix(".json").read_text())
            result["samples"] = report["samples"]
            result["authority"] = report["control_authorized"]
            if report["samples"] <= 0 or report["control_authorized"] or report["hardware_qualified"]:
                raise RuntimeError("No samples or invalid permission boundary")
        result["passed"] = code == expected and not any(token in error for token in (
            "AddressSanitizer", "LeakSanitizer", "runtime error:", "free():", "double free"))
        results.append(result)
        (args.output_dir / "summary.json").write_text(json.dumps({
            "read_only": True, "control_authorized": False,
            "passed": all(item["passed"] for item in results), "cases": results,
        }, indent=2) + "\n")
        print(f"{name}: exit={code}, expected={expected}, passed={result['passed']}", flush=True)
        if not result["passed"]:
            raise RuntimeError(f"Read-only probe exit regression failed: {prefix}")

    for index in range(args.cycles):
        run(f"repeat_{index:02d}", .5)
    run("full_ten_seconds", 10)
    run("sigint", 30, 128 + signal.SIGINT, signal.SIGINT)
    run("sigterm", 30, 128 + signal.SIGTERM, signal.SIGTERM)
    run("invalid_interface", .5, 1, interface="va15_no_iface")
    print(f"Read-only exit regression passed ({len(results)} cases); no motion acceptance.", flush=True)


if __name__ == "__main__":
    main()
