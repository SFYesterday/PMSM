# Basic FCS-MPC 实验

这里放 `pmsm.papers.basic_fcs_mpc` 的实验说明与验证结论。

推荐入口：

- `uv run python scripts/run_experiment.py --module pmsm.papers.basic_fcs_mpc`
- `uv run pytest tests/test_basic_fcs_mpc_accuracy.py -q`

结果图片：

- 运行实验后会自动生成 `results/figures/basic_fcs_mpc_results.png`

当前验证结论：

- 在低速工况（例如 `load_torque=4.0`, `T_sim=0.1`）下，`iq` 与 `id` 的稳态误差都可控制在约 `0.3 A` 以内。
- 默认配置长时间运行后会升速到较高转速，此时基础一拍 FCS-MPC 未包含弱磁或速度环，`iq` 跟踪会出现明显稳态偏差。
