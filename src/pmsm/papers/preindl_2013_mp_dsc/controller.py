"""MPC 核心算法、代价函数、观测器。"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from pmsm.common.motor_model import MotorParams, MotorState, electromagnetic_torque
from pmsm.common.transforms import switch_state_to_dq

SWITCH_STATES = np.array([
    [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
    [0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1],
])

ALLOWED_TRANSITIONS = {
    0: [0, 1, 3, 5],
    1: [1, 0, 2, 6],
    2: [2, 1, 3, 7],
    3: [3, 2, 0, 4],
    4: [4, 3, 5, 7],
    5: [5, 4, 0, 6],
    6: [6, 5, 1, 7],
    7: [7, 6, 2, 4],
}


@dataclass(frozen=True)
class ControllerParams:
    """MP-DSC 代价函数与观测器参数。"""

    lambda_T: float = 1.0
    lambda_A: float = 1e-3
    lambda_L: float = 1e4
    observer_Lp: float = 100.0
    observer_Li: float = 5000.0


@dataclass
class ControllerState:
    """控制器的内部记忆状态。"""

    omega_hat: float = 0.0
    tl_hat: float = 0.0
    last_state_idx: int = 0
    u_d_prev: float = 0.0
    u_q_prev: float = 0.0


@dataclass
class MPDSCController:
    """Preindl 2013 MP-DSC 控制器。"""

    motor_params: MotorParams = field(default_factory=MotorParams)
    controller_params: ControllerParams = field(default_factory=ControllerParams)
    state: ControllerState = field(default_factory=ControllerState)

    def step(self, motor_state: MotorState, w_ref: float) -> tuple[float, float, int]:
        """执行一次控制步，返回 (u_d, u_q, switch_index)。"""

        params = self.motor_params
        cparams = self.controller_params
        cstate = self.state

        # 负载观测器更新
        v_p = motor_state.omega_m - cstate.omega_hat
        te_est = electromagnetic_torque(motor_state, params)
        cstate.omega_hat += (
            (params.Ts / params.J)
            * (te_est - params.B_fric * cstate.omega_hat - cstate.tl_hat)
            + cparams.observer_Lp * v_p
        )
        cstate.tl_hat -= cparams.observer_Li * v_p * params.Ts
        cstate.omega_hat = float(np.clip(cstate.omega_hat, -5000.0, 5000.0))
        cstate.tl_hat = float(np.clip(cstate.tl_hat, -100.0, 100.0))

        # 延时补偿：先由上一拍电压估计 k 时刻状态
        omega_e = params.p * motor_state.omega_m
        id_k = motor_state.id + (params.Ts / params.Ld) * (
            cstate.u_d_prev - params.Rs * motor_state.id + omega_e * params.Lq * motor_state.iq
        )
        iq_k = motor_state.iq + (params.Ts / params.Lq) * (
            cstate.u_q_prev - params.Rs * motor_state.iq
            - omega_e * params.Ld * motor_state.id
            - omega_e * params.psi_pm
        )
        theta_e = motor_state.theta_e
        wm_k = cstate.omega_hat

        cost_min = float("inf")
        best_state_idx = cstate.last_state_idx
        best_ud = 0.0
        best_uq = 0.0

        # 仅遍历开关图允许的下一状态
        for idx in ALLOWED_TRANSITIONS[cstate.last_state_idx]:
            state = SWITCH_STATES[idx]
            ud_pred, uq_pred = switch_state_to_dq(state, params.Udc, theta_e)

            id_next = id_k + (params.Ts / params.Ld) * (
                ud_pred - params.Rs * id_k + omega_e * params.Lq * iq_k
            )
            iq_next = iq_k + (params.Ts / params.Lq) * (
                uq_pred - params.Rs * iq_k - omega_e * params.Ld * id_k - omega_e * params.psi_pm
            )
            te_next = 1.5 * params.p * (
                params.psi_pm * iq_next + (params.Ld - params.Lq) * id_next * iq_next
            )
            wm_next = wm_k + (params.Ts / params.J) * (te_next - params.B_fric * wm_k - cstate.tl_hat)

            # 数值保护，避免异常值污染寻优
            if not np.isfinite(wm_next) or not np.isfinite(id_next) or not np.isfinite(iq_next):
                continue

            c_t = min((wm_next - w_ref) ** 2, 1e12)
            c_a1 = min(
                (id_next + ((params.Ld - params.Lq) / params.psi_pm) * (id_next**2 - iq_next**2)) ** 2,
                1e12,
            )
            i_mag = (id_next**2 + iq_next**2) ** 0.5
            c_l1 = (i_mag - params.Ir_max) ** 2 if i_mag > params.Ir_max else 0.0
            limit_side = 2 * ((params.Ld - params.Lq) / params.psi_pm) * id_next + 1
            c_l2 = limit_side**2 if limit_side < 0 else 0.0
            cost = cparams.lambda_T * c_t + cparams.lambda_A * c_a1 + cparams.lambda_L * (c_l1 + c_l2)

            if cost < cost_min:
                cost_min = cost
                best_state_idx = idx
                best_ud = ud_pred
                best_uq = uq_pred

        cstate.last_state_idx = best_state_idx
        cstate.u_d_prev = best_ud
        cstate.u_q_prev = best_uq
        return best_ud, best_uq, best_state_idx
