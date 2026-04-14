# Basic FCS-MPC for PMSM

该模块是与 `preindl_2013_mp_dsc` 解耦的**独立基础 FCS-MPC**实现。

- `controller.py`：一步预测、8开关矢量枚举、电流误差代价函数
- `simulation.py`：基础仿真入口与波形记录
- `config.py`：结构化配置加载（对应 `configs/basic_fcs_mpc/fcs_mpc.toml`）

## API

- `FCSMPCController`
- `run_simulation()`

默认配置文件路径：`configs/basic_fcs_mpc/fcs_mpc.toml`。
