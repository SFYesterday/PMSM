"""MP-DSC 核心算法、代价函数与负载扰动观测。"""

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
    voltage_safety_factor: float = 0.95


@dataclass
class ControllerState:
    """控制器内部状态。"""

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

    def _update_disturbance_observer(self, motor_state: MotorState, te_est: float) -> None:
        params = self.motor_params
        cparams = self.controller_params
        cstate = self.state

        a_m = 1.0 - params.B_fric * params.Ts / params.J
        b_m = params.Ts / params.J
        # Scale the paper gains to the discrete-time observer used here.
        l_p = cparams.observer_Lp * params.Ts
        l_i = cparams.observer_Li * params.Ts

        omega_pred = a_m * cstate.omega_hat + b_m * (te_est - cstate.tl_hat)
        error = motor_state.omega_m - omega_pred

        cstate.omega_hat = float(np.clip(omega_pred + l_p * error, -5_000.0, 5_000.0))
        cstate.tl_hat = float(np.clip(cstate.tl_hat - l_i * error, -100.0, 100.0))

    def _attraction_cost(self, id_next: float, iq_next: float, omega_e_abs: float) -> float:
        params = self.motor_params
        cparams = self.controller_params

        mtpa_axis = (params.Ld - params.Lq) / params.psi_pm
        c_a1 = (id_next + mtpa_axis * (id_next**2 - iq_next**2)) ** 2

        voltage_margin = (
            (params.Lq * iq_next) ** 2
            + (params.Ld * id_next + params.psi_pm) ** 2
            - cparams.voltage_safety_factor * params.Udc / (np.sqrt(3.0) * max(omega_e_abs, 1e-9))
        )
        c_a2 = voltage_margin**2

        left_of_mtpa = 2.0 * mtpa_axis * id_next + 1.0 < 0.0
        if left_of_mtpa and c_a2 < c_a1:
            return float(c_a2)
        return float(c_a1)

    def _limitation_cost(self, id_next: float, iq_next: float, omega_e_abs: float) -> float:
        params = self.motor_params
        cparams = self.controller_params

        i_mag = float(np.hypot(id_next, iq_next))
        c_l1 = (i_mag - params.Ir_max) ** 2 if i_mag > params.Ir_max else 0.0

        mtpa_side = 2.0 * ((params.Ld - params.Lq) / params.psi_pm) * id_next + 1.0
        c_l2 = mtpa_side**2 if mtpa_side < 0.0 else 0.0

        xi = (
            np.hypot(params.Lq * iq_next, params.Ld * id_next + params.psi_pm)
            - cparams.voltage_safety_factor * params.Udc / (np.sqrt(3.0) * max(omega_e_abs, 1e-9))
        )
        c_l3 = xi**2 if xi > 0.0 else 0.0

        return float(c_l1 + c_l2 + c_l3)

    def step(self, motor_state: MotorState, w_ref: float) -> tuple[float, float, int]:
        """执行一次控制步，返回 `(u_d, u_q, switch_index)`。"""

        params = self.motor_params
        cparams = self.controller_params
        cstate = self.state

        te_k = electromagnetic_torque(motor_state, params)
        self._update_disturbance_observer(motor_state, te_k)

        omega_e = params.p * motor_state.omega_m
        theta_e = motor_state.theta_e

        # Past-input compensation: obtain x_e(k) using the previously applied voltage.
        id_k = motor_state.id + (params.Ts / params.Ld) * (
            cstate.u_d_prev - params.Rs * motor_state.id + omega_e * params.Lq * motor_state.iq
        )
        iq_k = motor_state.iq + (params.Ts / params.Lq) * (
            cstate.u_q_prev - params.Rs * motor_state.iq
            - omega_e * params.Ld * motor_state.id
            - omega_e * params.psi_pm
        )

        # The paper evaluates the speed that is actually affected by u(k), i.e. omega(k+2).
        wm_k1 = motor_state.omega_m + (params.Ts / params.J) * (
            te_k - params.B_fric * motor_state.omega_m - cstate.tl_hat
        )

        cost_min = float("inf")
        best_state_idx = cstate.last_state_idx
        best_ud = cstate.u_d_prev
        best_uq = cstate.u_q_prev

        for idx in ALLOWED_TRANSITIONS[cstate.last_state_idx]:
            switch_state = SWITCH_STATES[idx]
            ud_pred, uq_pred = switch_state_to_dq(switch_state, params.Udc, theta_e)

            id_k1 = id_k + (params.Ts / params.Ld) * (
                ud_pred - params.Rs * id_k + omega_e * params.Lq * iq_k
            )
            iq_k1 = iq_k + (params.Ts / params.Lq) * (
                uq_pred - params.Rs * iq_k - omega_e * params.Ld * id_k - omega_e * params.psi_pm
            )
            te_k1 = 1.5 * params.p * (
                params.psi_pm * iq_k1 + (params.Ld - params.Lq) * id_k1 * iq_k1
            )
            wm_k2 = wm_k1 + (params.Ts / params.J) * (
                te_k1 - params.B_fric * wm_k1 - cstate.tl_hat
            )

            if not np.isfinite(id_k1) or not np.isfinite(iq_k1) or not np.isfinite(wm_k2):
                continue

            omega_e_next_abs = abs(params.p * wm_k2)
            c_t = (wm_k2 - w_ref) ** 2
            c_a = self._attraction_cost(id_k1, iq_k1, omega_e_next_abs)
            c_l = self._limitation_cost(id_k1, iq_k1, omega_e_next_abs)
            cost = cparams.lambda_T * c_t + cparams.lambda_A * c_a + cparams.lambda_L * c_l

            if cost < cost_min:
                cost_min = float(cost)
                best_state_idx = idx
                best_ud = float(ud_pred)
                best_uq = float(uq_pred)

        cstate.last_state_idx = best_state_idx
        cstate.u_d_prev = best_ud
        cstate.u_q_prev = best_uq
        return best_ud, best_uq, best_state_idx
