# PMSM_Reproductions

这是一个用于复现 PMSM 相关论文的学习仓库。当前首个实现是 Preindl & Bolognani 2013 的 MP-DSC 示例，后续会继续按论文扩展。

## 环境

本项目使用 `uv` 管理依赖。

## 安装

```bash
uv sync
```

## 运行

```bash
uv run pmsm-sim
```

也可以直接运行兼容入口：

```bash
uv run python MP-DSC.py
```

通用实验脚本也可以这样用：

```bash
uv run python scripts/run_experiment.py --module pmsm.papers.preindl_2013_mp_dsc
```

## 项目结构（当前）

```text
PMSM_Reproductions/
├── src/
│   └── pmsm/
│       ├── __init__.py
│       ├── common/
│       │   ├── __init__.py
│       │   ├── motor_model.py
│       │   ├── transforms.py
│       │   └── plot_utils.py
│       ├── papers/
│       │   ├── __init__.py
│       │   ├── preindl_2013_mp_dsc/
│       │   │   ├── __init__.py
│       │   │   ├── controller.py
│       │   │   ├── simulation.py
│       │   │   └── README.md
│       ├── core/
│       ├── utils/
│       └── visualization/
├── docs/
│   ├── README.md
│   ├── notes/
│   │   └── README.md
│   └── papers/
│       ├── README.md
│       ├── preindl_2013_mp_dsc/
│       │   ├── README.md
│       │   └── notes.md
├── configs/
│   ├── preindl_2013_mp_dsc/
│   │   └── mp_dsc.toml
├── experiments/
│   ├── README.md
│   ├── preindl_2013_mp_dsc/
│   │   └── README.md
├── scripts/
│   └── run_experiment.py
├── tests/
│   ├── README.md
│   └── test_common.py
├── MP-DSC.py
├── pmsm_sim.py
├── pyproject.toml
├── uv.lock
├── README.md
├── LICENSE
└── .gitignore
```

## 新增论文的建议

1. 在 `docs/papers/<paper_id>/` 写笔记
2. 在 `configs/<paper_id>/` 放参数
3. 在 `src/pmsm/papers/<paper_id>/` 放实现
4. 在 `experiments/<paper_id>/` 写运行说明
