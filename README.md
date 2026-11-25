# Drone-assisted blood delivery

Code and inputs for **Benchmarking Drone-Assisted Blood Delivery after Disasters: The Joint Effects of Cargo Capacity and Ground Preparation**.

## Reproduce

The recorded environment is Python 3.9.6. Install the pinned dependencies and run:

```sh
python -m pip install -r requirements.txt
python reproduce.py recompute
```

This rebuilds the geography, demand and road graph; runs model checks; solves the drone-allocation and resource experiments; and compares the outputs with the supplied reference results. New outputs and logs go into `runs/`, with a `RUN_REPORT.json` summary.

Two shorter commands are available:

- `python reproduce.py verify`: check file integrity without numerical dependencies.
- `python reproduce.py check`: rebuild inputs and reproduce the ground-delivery and load-dependent preparation comparisons using retained flight plans.

No network connection is needed after installing dependencies. Alternative optimal assignments and runtimes may differ.

## Contents

| Path | Purpose |
|---|---|
| `src/` | Geography, demand, road and drone models; ground comparison; resource and preparation experiments; independent checks |
| `data/inputs.zip` | Essential scenario inputs, one road-source extract and retained decisions; unpacked automatically |
| `data/sources.csv`, `ATTRIBUTION.txt` | Source records and data attribution |
| `expected/` | Compact numerical targets for checking results |
| `reproduce.py`, `requirements.txt`, `MANIFEST.json` | Run commands, dependency versions and file integrity |

Detailed schedules, solver logs and figures are generated locally.

## Optional runs

To draw the vehicle and drone network solution (Figure 2) after a completed run:

```sh
python -m pip install -r requirements-figures.txt
python plot_network_solution.py --run-dir runs/<run-id>
```

Replace `runs/<run-id>` with the completed run directory. The script checks the recorded decisions and exports PDF, PNG and SVG files into `figure_outputs/`.

`python reproduce.py resolve-roads` performs new time-limited interval-regret road solves and the closure experiment. Selected paths, bounds and downstream results may differ; the other commands use retained road decisions.

The experiments are conditional planning benchmarks with unequal hybrid and ground resource packages. They do not establish economic superiority or operational validation. Load-dependent preparation optimizes crew schedules for retained flight and quantity plans. Computation starts from curated inputs; source collection and field validation are outside its scope.
