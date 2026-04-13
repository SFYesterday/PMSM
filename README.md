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

## 当前结构

- `src/pmsm/common/`：所有论文共享的基建代码
- `src/pmsm/papers/`：按论文隔离的复现代码
- `docs/`：论文原文与推导笔记
- `tests/`：基础测试
- `configs/`：论文参数配置
- `experiments/`：实验说明
- `results/`：仿真结果输出
- `scripts/`：通用运行脚本

### 当前首篇论文

- `src/pmsm/papers/preindl_2013_mp_dsc/`：Preindl 2013 MP-DSC
- `src/pmsm/common/motor_model.py`：PMSM 电机物理本体模型
- `src/pmsm/common/transforms.py`：Clarke/Park 变换
- `src/pmsm/common/plot_utils.py`：统一画图工具

## 新增论文的建议

1. 在 `docs/papers/<paper_id>/` 写笔记
2. 在 `configs/<paper_id>/` 放参数
3. 在 `src/pmsm/papers/<paper_id>/` 放实现
4. 在 `experiments/<paper_id>/` 写运行说明
