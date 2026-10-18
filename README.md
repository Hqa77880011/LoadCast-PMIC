# LoadCast-PMIC

Implementation of **Gated workload-predictive feedforward control for edge-AI power delivery**.

Implements exact zero-order-hold discretization of the averaged buck plant, saturated PID with anti-windup, bounded predictive feedforward and the paper's residual-variance gate.

## Install

Use Python 3.11 or newer.

```bash
git clone https://github.com/Hqa77880011/LoadCast-PMIC.git
cd LoadCast-PMIC
python -m venv .venv
```

Activate with `source .venv/bin/activate` on Linux/macOS or `.venv\Scripts\Activate.ps1` in PowerShell, then install.

```bash
python -m pip install -e .
```

## Data preparation

Load and forecast arrays share one switching-cycle grid with paired sensor-noise samples. Plant/controller fields specify Vin, reference, L, C, ESR, frequency, duty limits, feedback gains, feedforward gain and gate scale.

```bash
python -m loadcast prepare --count 200 --seed 17 --output data/workload.jsonl
```

The generator creates finite workloads from the formal model. Each JSONL row is one scenario. `--count` controls corpus size and `--seed` controls generation. Imported cases use the same schema. Generate calibration and test scenarios independently.

## Execute and compare

```bash
python -m loadcast run --data data/workload.jsonl --output runs/main
python -m loadcast suite --data data/workload.jsonl --output runs/comparison
```

Select methods with `--methods`. gated follows the residual gate; pid disables preview; static_ff uses present load estimate; no_gate keeps preview; observable_gate estimates load from sensed current and voltage changes; oracle_mpc receives true four-cycle future load.

## Evaluation and visualization

Worst droop/overshoot, RMS voltage error, peak-to-peak ripple, settling to a 5 mV band, unsettled transitions, 0.84 V violation and estimated ESR-loss efficiency.

Each method writes `records.jsonl` and `metrics.json` with means, sample standard deviations, usable counts and P50/P95/P99. Rates are fractions. Undefined conditional metrics remain unavailable. Method timers exclude reference scoring. RSS includes the Python runtime; serialized sizes measure JSON bytes.

```bash
python -m loadcast plot --inputs runs/comparison/gated --output figures
```

Pass multiple method directories to compare them. Plotting writes `summary.csv` and metric PNGs.

## Implementation conventions

The default plant uses the paper's 3.3 V input, 0.9 V reference, 2 MHz switching, 0.47 uH, 47 uF and 8 mOhm ESR. The evaluated residual gate uses simulated true current and is an offline diagnostic; observable_gate is a separate controller. Efficiency accounts for ESR loss, without inventing unavailable transistor-loss parameters. Switching-node waveforms are outside the averaged model.


## Uncertainty and paired comparisons

```bash
python -m loadcast.statistics --records runs/main/gated/records.jsonl --output results/intervals.json
```

Supply --reference with a matched record file for paired differences. Resampling uses independent case IDs, or explicit cluster IDs when provided. Repeated observations from one source unit must share cluster. Zero-event bounds refer to an event in an independent cluster.

## Fixed reproduction setting

Averaged 3.3V/0.9V 2MHz buck model with generated load/forecast/noise; compare implementable observable gate and offline true-load gate separately.

The checked-in entry point fixes data seeds, methods, budgets and output paths. Inspect its commands without starting any computation:

```bash
python reproduce.py --plan
```

Remove `--plan` to execute the registered pipeline in a fresh output directory. The entry point calls the existing implementation; it does not insert paper results. Installation uses the package instructions above.
