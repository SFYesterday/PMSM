# PMSM Reproductions

English | [中文](./README_zh.md)

This repository is a Python project for reproducing and validating control algorithms for PMSM (Permanent Magnet Synchronous Motor) drives.

It currently contains two main tracks:

- `basic_fcs_mpc`: a standalone, minimal baseline implementation of finite control set model predictive current control (FCS-MPC).
- `preindl_2013_mp_dsc`: a reproduction of the MP-DSC algorithm from Preindl & Bolognani (2013).

The goal is not only to “translate equations into code”, but to keep a practical engineering pipeline:

**paper -> implementation -> experiment -> verification**

## Quick Start

### Install dependencies

```bash
uv sync
```

### Run experiments

Default experiment entry:

```bash
uv run python scripts/run_experiment.py
```

Run a specific module:

```bash
uv run python scripts/run_experiment.py --module pmsm.papers.basic_fcs_mpc
uv run python scripts/run_experiment.py --module pmsm.papers.preindl_2013_mp_dsc
```

### Run tests

```bash
uv run pytest -q
```

## Repository Structure

```text
PMSM/
├── src/pmsm/
│   ├── common/
│   ├── papers/
│   │   ├── basic_fcs_mpc/
│   │   └── preindl_2013_mp_dsc/
│   ├── core/
│   ├── utils/
│   └── visualization/
├── configs/
├── docs/
├── experiments/
├── tests/
├── scripts/
└── results/
```

## Core Components

- `src/pmsm/common/motor_model.py`  
  PMSM discrete-time plant model (`MotorParams`, `MotorState`, `step_motor_state`).

- `src/pmsm/common/transforms.py`  
  Clarke/Park transforms and switch-state-to-`dq` mapping.

- `src/pmsm/common/plot_utils.py`  
  Plotting utilities for both `basic_fcs_mpc` and `preindl_2013_mp_dsc` outputs.

## Current Status

- `basic_fcs_mpc` is a clean and extensible baseline for current-loop predictive control.
- `preindl_2013_mp_dsc` includes a richer speed-control objective with observer and constraints.
- Unit tests are included to lock in expected behavior and reduce regression risk.

## Suggested Workflow for New Papers

1. Add notes under `docs/papers/<paper_id>/`.
2. Add configs under `configs/<paper_id>/`.
3. Add implementation under `src/pmsm/papers/<paper_id>/`.
4. Add experiment notes under `experiments/<paper_id>/`.
5. Add tests under `tests/`.

This keeps the project consistent and scalable as more papers are added.
