"""基础一拍预测 FCS-MPC 电流控制器。"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from pmsm.common.motor_model import MotorParams, MotorState
from pmsm.common.transforms import switch_state_to_dq

SWITCH_STATES = np.array([
    [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
    [0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1],
])


@dataclass(frozen=True)
class FCSMPCParams:
    """FCS-MPC 权重参数。"""

    lambda_id: float = 1.0
    lambda_iq: float = 1.0
    lambda_sw: float = 0.0
    id_ref_default: float = 0.0


@dataclass
class FCSMPCState:
    """FCS-MPC 内部记忆状态。"""

    last_state_idx: int = 0


@dataclass
class FCSMPCController:
    """基础 FCS-MPC（finite control set）电流控制器。"""

    motor_params: MotorParams = field(default_factory=MotorParams)
    mpc_params: FCSMPCParams = field(default_factory=FCSMPCParams)
    state: FCSMPCState = field(default_factory=FCSMPCState)

    def step(self, motor_state: MotorState, iq_ref: float, id_ref: float | None = None) -> tuple[float, float, int]:
        """执行一次控制步，返回 (u_d, u_q, switch_index)。"""

        params = self.motor_params
        cparams = self.mpc_params
        cstate = self.state

        if id_ref is None:
            id_ref = cparams.id_ref_default

        omega_e = params.p * motor_state.omega_m
        theta_e = motor_state.theta_e

        prev_state = SWITCH_STATES[cstate.last_state_idx]
        best_idx = cstate.last_state_idx
        best_ud = 0.0
        best_uq = 0.0
        min_cost = float("inf")

        for idx, sw in enumerate(SWITCH_STATES):
            ud_pred, uq_pred = switch_state_to_dq(sw, params.Udc, theta_e)

            id_next = motor_state.id + (params.Ts / params.Ld) * (
                ud_pred - params.Rs * motor_state.id + omega_e * params.Lq * motor_state.iq
            )
            iq_next = motor_state.iq + (params.Ts / params.Lq) * (
                uq_pred - params.Rs * motor_state.iq - omega_e * params.Ld * motor_state.id - omega_e * params.psi_pm
            )

            current_cost = (
                cparams.lambda_id * (id_ref - id_next) ** 2
                + cparams.lambda_iq * (iq_ref - iq_next) ** 2
            )
            switch_cost = cparams.lambda_sw * float(np.sum(np.abs(sw - prev_state)))
            cost = current_cost + switch_cost

            if cost < min_cost:
                min_cost = cost
                best_idx = idx
                best_ud = ud_pred
                best_uq = uq_pred

        cstate.last_state_idx = best_idx
        return float(best_ud), float(best_uq), int(best_idx)
