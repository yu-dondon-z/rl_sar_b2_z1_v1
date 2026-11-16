#!/usr/bin/env python3
"""Plot the default-friction paired simulation; no viewer or robot access."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    directory = parser.parse_args().directory
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    for kind, color in (("legacy", "#b66a41"), ("candidate", "#137e96")):
        with (directory / f"{kind}_mu0.8_tau1.0.csv").open() as f:
            rows = list(csv.DictReader(f))
        short = [r for r in rows if 5.9 <= float(r["time"]) <= 8.0]
        motion = [r for r in rows if 5.9 <= float(r["time"]) <= 12.0]
        for ax, subset, key, scale in ((axes[0, 0], short, "truth_h", 100),
                                        (axes[0, 1], short, "truth_tilt", 57.2957795),
                                        (axes[1, 0], motion, "forward_displacement", 100),
                                        (axes[1, 1], motion, "lateral_displacement", 100)):
            ax.plot([float(r["time"]) - 6 for r in subset], [float(r[key]) * scale for r in subset],
                    label=kind, color=color, linewidth=1.6)
    for ax, title, ylabel in ((axes[0, 0], "Startup body height", "Height (cm)"),
                              (axes[0, 1], "Startup body tilt", "Tilt (deg)"),
                              (axes[1, 0], "Forward travel and stop", "Forward (cm)"),
                              (axes[1, 1], "Lateral drift (NOT fixed)", "Lateral (cm)")):
        ax.set(title=title, ylabel=ylabel, xlabel="Time since forward request (s)")
        ax.grid(alpha=.25)
        ax.legend()
        ax.axvline(0, color="gray", linestyle=":", alpha=.6)
    for ax in axes[1]:
        ax.axvline(4, color="gray", linestyle="--", alpha=.6)
    fig.suptitle("VA15 startup-only V3 candidate vs V2 join | MuJoCo, no gun/hose, 0.15 m/s request\n"
                 "Same policy/gains/stop gates; simulation evidence, NOT hardware acceptance", fontsize=11)
    target = directory / "startup_comparison.png"
    fig.savefig(target, dpi=150)
    print(target)


if __name__ == "__main__":
    main()
