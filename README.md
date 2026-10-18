# LoadCast-PMIC

Implementation of **Gated workload-predictive feedforward control for edge-AI power delivery**.

## Installation

Requires Python `>=3.11`. Run from the repository root in an activated environment.

```bash
python -m pip install -e .
```

Inspection environment: Python 3.13.5; numpy 2.1.3, scipy 1.15.3, scikit-learn 1.6.1, matplotlib 3.10.0. Checks: Python syntax, configuration parsing and CLI/source argument matching. Training and full evaluation were not run.

## Data

Load and forecast arrays share one switching-cycle grid with paired sensor-noise samples. Plant/controller fields specify Vin, reference, L, C, ESR, frequency, duty limits, feedback gains, feedforward gain and gate scale.

The bundled generator writes synthetic finite-model scenarios as JSONL. Fix the seed and keep the generated input for paired comparisons.

```bash
python -m loadcast prepare --count 200 --seed 17 --output data/generated.jsonl
```

`gated` uses simulated true load for its residual and is an offline diagnostic; `observable_gate` uses the observable current estimate.

## Execution and evaluation

```bash
python -m loadcast run --data data/generated.jsonl --output runs/main
```

## Comparisons and ablations

```bash
python -m loadcast suite --data data/generated.jsonl --output runs/comparison
```

Use `--methods` to select: `gated`, `pid`, `static_ff`, `no_gate`, `observable_gate`, `oracle_mpc`.

## Outputs

`runs/main/gated/records.jsonl` stores per-case results; `metrics.json` stores aggregate measurements. Comparison outputs use one directory per method.

```bash
python -m loadcast plot --inputs runs/comparison/gated --output figures
```

Plotting writes `figures/summary.csv` and metric PNGs. Statistical intervals can be computed from a retained record file:

```bash
python -m loadcast.statistics --records runs/main/gated/records.jsonl --output results/intervals.json
```
