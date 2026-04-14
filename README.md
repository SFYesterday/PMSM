# PMSM Reproductions

这是一个面向 PMSM（永磁同步电机）控制算法复现与验证的 Python 项目。仓库当前包含两条主线：

- `basic_fcs_mpc`：一个独立、最小化的基础有限控制集模型预测电流控制（FCS-MPC）基线实现。
- `preindl_2013_mp_dsc`：基于 Preindl 与 Bolognani 2013 论文的 MP-DSC（Model Predictive Direct Speed Control）复现实现。

项目目标不是只把公式“翻译成代码”，而是把论文算法拆成可运行、可验证、可扩展的工程结构，方便继续做参数试验、算法对比和后续增强。

## 你能用这个项目做什么

- 直接运行 PMSM 预测控制仿真。
- 对照论文场景验证算法行为是否合理。
- 查看控制器、电机模型、坐标变换、绘图工具在代码里的对应关系。
- 在统一框架下继续添加新的论文算法实现。

## 快速开始

### 安装依赖

```bash
uv sync
```

### 运行默认实验

当前通用入口默认运行 `basic_fcs_mpc`：

```bash
uv run python scripts/run_experiment.py
```

也可以显式指定模块：

```bash
uv run python scripts/run_experiment.py --module pmsm.papers.basic_fcs_mpc
uv run python scripts/run_experiment.py --module pmsm.papers.preindl_2013_mp_dsc
```

### 运行测试

```bash
uv run pytest -q
```

### 查看结果图

实验运行后会把图片保存到：

- `results/figures/basic_fcs_mpc_results.png`
- `results/figures/preindl_2013_mp_dsc_results.png`

## 项目整体结构

```text
PMSM/
├── src/pmsm/
│   ├── common/
│   │   ├── motor_model.py
│   │   ├── transforms.py
│   │   └── plot_utils.py
│   ├── papers/
│   │   ├── basic_fcs_mpc/
│   │   └── preindl_2013_mp_dsc/
│   ├── core/
│   ├── utils/
│   └── visualization/
├── configs/
│   ├── basic_fcs_mpc/
│   └── preindl_2013_mp_dsc/
├── experiments/
├── docs/
├── tests/
├── scripts/
└── results/
```

可以把它理解成 5 层：

1. `common/`
   这里放所有算法都会复用的基础能力，比如 PMSM 数学模型、坐标变换和绘图。
2. `papers/`
   每篇论文一个独立目录，里面放该算法自己的控制器与仿真编排。
3. `configs/`
   存放算法或实验的参数配置。
4. `experiments/` 和 `docs/`
   前者偏“怎么跑实验”，后者偏“论文笔记、复现记录、背景说明”。
5. `tests/`
   通过单测和行为测试把“算法是否工作正常”固定下来。

## 代码是怎么跑起来的

从命令行执行：

```bash
uv run python scripts/run_experiment.py --module pmsm.papers.preindl_2013_mp_dsc
```

实际流程是：

1. [scripts/run_experiment.py](D:/Project/Python_Pro/PMSM/scripts/run_experiment.py) 根据 `--module` 动态导入对应论文模块。
2. 该模块的 `main()` 被调用。
3. `main()` 一般会调用 `run_simulation()` 完成整段离散时间仿真。
4. 仿真内部每个采样周期都会：
   - 读取当前电机状态
   - 调用控制器 `step(...)`
   - 得到本拍的 `u_d / u_q` 或开关状态
   - 用电机模型推进到下一个采样时刻
   - 记录速度、电流、转矩、观测量等波形
5. 仿真结束后，结果会被绘图函数画成图并保存到 `results/figures/`。

## 公共模块详解

### [src/pmsm/common/motor_model.py](D:/Project/Python_Pro/PMSM/src/pmsm/common/motor_model.py)

这是整个项目的“被控对象”。

核心内容：

- `MotorParams`
  电机和仿真参数集合，包括：
  - `Ts`：采样周期
  - `Udc`：直流母线电压
  - `Ir_max`：电流幅值限制
  - `Ld`、`Lq`：dq 轴电感
  - `Rs`：定子电阻
  - `psi_pm`：永磁体磁链
  - `p`：极对数
  - `J`：转动惯量
  - `B_fric`：粘性摩擦系数

- `MotorState`
  表示系统在某一采样时刻的状态：
  - `id`、`iq`：dq 轴电流
  - `omega_m`：机械角速度
  - `theta_e`：电角度

- `electromagnetic_torque(...)`
  根据当前状态计算电磁转矩。

- `step_motor_state(...)`
  这是仿真最关键的推进函数。它用离散时间方式，把输入电压 `ud/uq` 和负载转矩 `load_torque` 转成下一拍电流、速度与角度。

如果把整个项目看成“控制器 + 电机”的闭环系统，这个文件就是电机那一侧。

### [src/pmsm/common/transforms.py](D:/Project/Python_Pro/PMSM/src/pmsm/common/transforms.py)

这个文件负责三相开关状态与 `dq` 坐标系电压之间的映射。

包含三层变换：

- `clarke_transform(...)`
  把三相逆变器开关状态映射到静止坐标系 `alpha-beta` 电压。

- `park_transform(...)`
  把 `alpha-beta` 电压旋转到电机同步旋转坐标系 `dq`。

- `switch_state_to_dq(...)`
  项目里最常用的封装函数。给定一个三相开关向量、母线电压和电角度，直接得到 `u_d` 与 `u_q`。

这一步是 FCS-MPC 一类算法的基础，因为它们本质上是在枚举逆变器的有限开关状态。

### [src/pmsm/common/plot_utils.py](D:/Project/Python_Pro/PMSM/src/pmsm/common/plot_utils.py)

这个文件负责把仿真结果整理成实验图。

目前主要有两个绘图函数：

- `plot_basic_fcs_mpc_results(...)`
  绘制基础 FCS-MPC 的电流跟踪、误差、转速/转矩、开关状态。

- `plot_mp_dsc_results(...)`
  绘制 MP-DSC 的速度响应、负载观测、电流轨迹和 MTPA 曲线。

现在两个函数都支持：

- `save_path`
  保存图片到指定路径。
- `show`
  是否弹出图窗。

这让项目既能在本地交互运行，也能在无界面终端里稳定保存结果。

## 论文模块详解

### 1. [src/pmsm/papers/basic_fcs_mpc/](D:/Project/Python_Pro/PMSM/src/pmsm/papers/basic_fcs_mpc)

这是一个基础版电流控制器，目的不是尽可能复杂，而是提供一个简单、可对照、可扩展的预测控制基线。

主要文件：

- [controller.py](D:/Project/Python_Pro/PMSM/src/pmsm/papers/basic_fcs_mpc/controller.py)
  实现一拍 FCS-MPC 控制器。
  它会遍历 8 个逆变器开关状态，预测下一拍 `id/iq`，再根据电流误差代价选出最优开关。

- [config.py](D:/Project/Python_Pro/PMSM/src/pmsm/papers/basic_fcs_mpc/config.py)
  负责从 `configs/basic_fcs_mpc/fcs_mpc.toml` 读取参数，转成结构化配置对象。

- [simulation.py](D:/Project/Python_Pro/PMSM/src/pmsm/papers/basic_fcs_mpc/simulation.py)
  把控制器和电机模型接起来，执行完整时域仿真，并保存实验图。

这个模块当前的验证结论是：

- 低速工况下，`iq` 跟踪表现良好。
- 默认长时间工况下，由于没有弱磁和速度外环，进入高转速后会出现明显稳态误差。

也就是说，它更像“基础算法底座”，而不是完整工业控制方案。

### 2. [src/pmsm/papers/preindl_2013_mp_dsc/](D:/Project/Python_Pro/PMSM/src/pmsm/papers/preindl_2013_mp_dsc)

这是论文 `Model Predictive Direct Speed Control with Finite Control Set of PMSM Drive Systems` 的复现实现。

主要文件：

- [controller.py](D:/Project/Python_Pro/PMSM/src/pmsm/papers/preindl_2013_mp_dsc/controller.py)
  实现 MP-DSC 控制核心，包括：
  - 有限开关状态枚举
  - 开关图约束 `ALLOWED_TRANSITIONS`
  - 延迟补偿下的电流预测
  - 速度预测
  - 负载扰动观测器
  - 代价函数中的速度项、吸引域项和限制项

- [simulation.py](D:/Project/Python_Pro/PMSM/src/pmsm/papers/preindl_2013_mp_dsc/simulation.py)
  构建论文中的典型实验场景：
  - `0 ~ 0.05 s`：速度参考为 0
  - `0.05 s` 后：速度阶跃到 `500 rpm`
  - `0.2 s` 后：施加 `6 N·m` 负载扰动

这个模块是当前仓库里更接近“完整论文算法”的实现。它相较于基础 FCS-MPC，多了速度层目标、MTPA/约束相关代价和负载观测。

## 配置、实验、文档、测试分别是什么

### `configs/`

这里存放算法参数。

- `configs/basic_fcs_mpc/fcs_mpc.toml`
  基础 FCS-MPC 的仿真、电机、控制器、参考值配置。

- `configs/preindl_2013_mp_dsc/mp_dsc.toml`
  MP-DSC 的参数配置草稿。当前 `basic_fcs_mpc` 已完整接入配置加载，`mp_dsc` 未来也可以继续完全统一到这套结构化配置方式。

### `experiments/`

这里写“怎么运行某个实验、目前观察到了什么结果”。

它更像实验手册，而不是代码说明。

### `docs/`

这里放论文笔记、复现说明和资料整理。

适合记录：

- 论文核心公式
- 推导过程
- 和代码实现之间的映射关系
- 参数来源
- 对复现结果的解释

### `tests/`

这里不仅仅做“代码有没有报错”的单测，更重要的是固定住算法行为。

例如：

- `test_fcs_mpc.py`
  验证基础 FCS-MPC 至少能选出合法开关，并能从零电流把 `iq` 往正方向推。

- `test_basic_fcs_mpc_accuracy.py`
  验证 `basic_fcs_mpc` 在低速工况下跟踪有效，并明确记录默认工况下的高转速误差问题。

- `test_preindl_mp_dsc.py`
  验证 MP-DSC 能在 `500 rpm` 下稳定跟踪，并能对 `6 N·m` 负载扰动做出合理抑制和估计。

这类测试的价值很高，因为以后你继续改公式、调参数、重构代码时，可以立刻知道自己是不是把原有行为改坏了。

## 如果你想读代码，建议按这个顺序

1. 先看 [src/pmsm/common/motor_model.py](D:/Project/Python_Pro/PMSM/src/pmsm/common/motor_model.py)
   理解系统状态和离散推进方式。
2. 再看 [src/pmsm/common/transforms.py](D:/Project/Python_Pro/PMSM/src/pmsm/common/transforms.py)
   理解开关状态如何变成 `dq` 电压。
3. 再看 [src/pmsm/papers/basic_fcs_mpc/controller.py](D:/Project/Python_Pro/PMSM/src/pmsm/papers/basic_fcs_mpc/controller.py)
   先从最简单的预测控制器开始看。
4. 然后看 [src/pmsm/papers/preindl_2013_mp_dsc/controller.py](D:/Project/Python_Pro/PMSM/src/pmsm/papers/preindl_2013_mp_dsc/controller.py)
   看完整的速度预测、约束与观测器。
5. 最后看两个 `simulation.py`
   理解实验如何组织、波形如何记录、图如何生成。

## 后续扩展建议

如果你后面继续加新论文，建议按这个模板扩展：

1. 在 `docs/papers/<paper_id>/` 记录论文和笔记。
2. 在 `configs/<paper_id>/` 放参数。
3. 在 `src/pmsm/papers/<paper_id>/` 放实现。
4. 在 `experiments/<paper_id>/` 写实验说明。
5. 在 `tests/` 里补行为验证测试。

这样项目会一直保持“论文 -> 代码 -> 实验 -> 验证”这条链路是闭合的。
