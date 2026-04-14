"""基础 FCS-MPC 仿真编排。"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from pmsm.common.motor_model import MotorParams, MotorState, step_motor_state
from pmsm.common.plot_utils import plot_basic_fcs_mpc_results

from .controller import FCSMPCController
from .config import load_config


def run_simulation(
    config_path: str | Path | None = None,
    iq_ref: float | None = None,
    id_ref: float | None = None,
    load_torque: float | None = None,
    t_sim: float | None = None,
) -> dict[str, np.ndarray]:
    """运行基础 FCS-MPC 电流控制仿真并返回关键波形。"""

    config = load_config(config_path)

    sim_cfg = config.simulation
    motor_cfg = config.motor
    ctrl_cfg = config.controller
    ref_cfg = config.references

    motor_params = MotorParams(
        Ts=sim_cfg.Ts,
        Udc=sim_cfg.Udc,
        Ir_max=sim_cfg.Ir_max,
        Ld=motor_cfg.Ld,
        Lq=motor_cfg.Lq,
        Rs=motor_cfg.Rs,
        psi_pm=motor_cfg.psi_pm,
        p=motor_cfg.p,
        J=motor_cfg.J,
        B_fric=motor_cfg.B_fric,
    )
    controller = FCSMPCController(motor_params=motor_params)
    controller.mpc_params = controller.mpc_params.__class__(
        lambda_id=ctrl_cfg.lambda_id,
        lambda_iq=ctrl_cfg.lambda_iq,
        lambda_sw=ctrl_cfg.lambda_sw,
        id_ref_default=ctrl_cfg.id_ref_default,
    )

    iq_ref_eff = ref_cfg.iq_ref if iq_ref is None else iq_ref
    id_ref_eff = ref_cfg.id_ref if id_ref is None else id_ref
    load_torque_eff = sim_cfg.load_torque if load_torque is None else load_torque
    t_sim_eff = sim_cfg.T_sim if t_sim is None else t_sim

    n_steps = int(t_sim_eff / motor_params.Ts)
    time = np.arange(n_steps) * motor_params.Ts

    id_log = np.zeros(n_steps)
    iq_log = np.zeros(n_steps)
    w_log = np.zeros(n_steps)
    te_log = np.zeros(n_steps)
    id_ref_log = np.full(n_steps, id_ref_eff)
    iq_ref_log = np.full(n_steps, iq_ref_eff)
    switch_idx_log = np.zeros(n_steps, dtype=int)

    motor_state = MotorState()

    for k in range(n_steps):
        ud, uq, sw_idx = controller.step(motor_state, iq_ref=iq_ref_eff, id_ref=id_ref_eff)
        motor_state, torque = step_motor_state(motor_state, ud, uq, load_torque_eff, motor_params)

        id_log[k] = motor_state.id
        iq_log[k] = motor_state.iq
        w_log[k] = motor_state.omega_m * (60 / (2 * math.pi))
        te_log[k] = torque
        switch_idx_log[k] = sw_idx

    return {
        "time": time,
        "id_ref_log": id_ref_log,
        "iq_ref_log": iq_ref_log,
        "id_log": id_log,
        "iq_log": iq_log,
        "w_log": w_log,
        "Te_log": te_log,
        "switch_idx_log": switch_idx_log,
        "motor_params": motor_params,
    }


def main() -> None:
    """运行基础 FCS-MPC 仿真并保存结果图。"""

    results = run_simulation()
    figure_path = Path(__file__).resolve().parents[4] / "results" / "figures" / "basic_fcs_mpc_results.png"
    plot_basic_fcs_mpc_results(
        results["time"],
        results["id_ref_log"],
        results["iq_ref_log"],
        results["id_log"],
        results["iq_log"],
        results["w_log"],
        results["Te_log"],
        results["switch_idx_log"],
        save_path=figure_path,
        show=False,
    )
    print(
        "[basic_fcs_mpc] "
        f"id_end={results['id_log'][-1]:.3f}, "
        f"iq_end={results['iq_log'][-1]:.3f}, "
        f"speed_rpm_end={results['w_log'][-1]:.2f}, "
        f"figure={figure_path}"
    )


if __name__ == "__main__":
    main()
