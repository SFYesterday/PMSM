"""PMSM 电机物理本体的微分方程仿真模型。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MotorParams:
    """PMSM 与仿真的基础参数集合。"""

    Ts: float = 100e-6
    Udc: float = 200.0
    Ir_max: float = 10.0
    Ld: float = 12e-3
    Lq: float = 20e-3
    Rs: float = 0.636
    psi_pm: float = 0.088
    p: int = 5
    J: float = 1.0e-3
    B_fric: float = 1.7e-3


@dataclass
class MotorState:
    """PMSM 在 dq 坐标系与机械侧的状态量。"""

    id: float = 0.0
    iq: float = 0.0
    omega_m: float = 0.0
    theta_e: float = 0.0


def electromagnetic_torque(state: MotorState, params: MotorParams) -> float:
    """计算电磁转矩 Te。"""

    return 1.5 * params.p * (params.psi_pm * state.iq + (params.Ld - params.Lq) * state.id * state.iq)


def step_motor_state(
    state: MotorState,
    ud: float,
    uq: float,
    load_torque: float,
    params: MotorParams,
) -> tuple[MotorState, float]:
    """离散时间推进一步电机状态，返回新状态与该步转矩。"""

    # 电角速度
    omega_e = params.p * state.omega_m

    # dq 轴电流微分方程
    did_dt = (ud - params.Rs * state.id + omega_e * params.Lq * state.iq) / params.Ld
    diq_dt = (uq - params.Rs * state.iq - omega_e * params.Ld * state.id - omega_e * params.psi_pm) / params.Lq

    # 积分得到下一时刻电流
    new_id = state.id + params.Ts * did_dt
    new_iq = state.iq + params.Ts * diq_dt
    new_state = MotorState(id=new_id, iq=new_iq, omega_m=state.omega_m, theta_e=state.theta_e)

    # 机械方程：由转矩差更新机械角速度
    torque = electromagnetic_torque(new_state, params)
    domega_dt = (torque - params.B_fric * state.omega_m - load_torque) / params.J
    new_omega_m = state.omega_m + params.Ts * domega_dt

    # 电角度更新并归一化
    new_theta_e = (state.theta_e + params.Ts * omega_e) % (2 * np.pi)

    return MotorState(id=new_id, iq=new_iq, omega_m=new_omega_m, theta_e=new_theta_e), torque
