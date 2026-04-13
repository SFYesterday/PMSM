"""组装 common 里的电机和此处的 controller 进行仿真。"""

from __future__ import annotations

import math

import numpy as np

from pmsm.common.motor_model import MotorParams, MotorState, step_motor_state
from pmsm.common.plot_utils import plot_mp_dsc_results

from .controller import MPDSCController


def run_simulation() -> dict[str, np.ndarray]:
    """运行完整 MP-DSC 仿真并返回关键波形。"""

    motor_params = MotorParams()
    controller = MPDSCController(motor_params=motor_params)

    T_sim = 0.4
    N_steps = int(T_sim / motor_params.Ts)
    time = np.arange(N_steps) * motor_params.Ts

    id_log = np.zeros(N_steps)
    iq_log = np.zeros(N_steps)
    w_log = np.zeros(N_steps)
    Te_log = np.zeros(N_steps)
    w_ref_log = np.zeros(N_steps)
    Tl_hat_log = np.zeros(N_steps)

    motor_state = MotorState()

    # 主仿真循环
    for k in range(N_steps):
        w_ref = 0.0 if time[k] < 0.05 else 500.0 * (2 * math.pi / 60)
        load_torque = 0.0 if time[k] < 0.2 else 6.0
        w_ref_log[k] = w_ref

        ud, uq, _ = controller.step(motor_state, w_ref)
        motor_state, torque = step_motor_state(motor_state, ud, uq, load_torque, motor_params)

        id_log[k] = motor_state.id
        iq_log[k] = motor_state.iq
        w_log[k] = motor_state.omega_m * (60 / (2 * math.pi))
        Te_log[k] = torque
        Tl_hat_log[k] = controller.state.tl_hat

    return {
        "time": time,
        "w_ref_log": w_ref_log,
        "w_log": w_log,
        "Te_log": Te_log,
        "Tl_hat_log": Tl_hat_log,
        "id_log": id_log,
        "iq_log": iq_log,
        "motor_params": motor_params,
    }


def main() -> None:
    """运行仿真并绘图。"""

    results = run_simulation()
    plot_mp_dsc_results(
        results["time"],
        results["w_ref_log"],
        results["w_log"],
        results["Te_log"],
        results["Tl_hat_log"],
        results["id_log"],
        results["iq_log"],
        results["motor_params"],
    )


if __name__ == "__main__":
    main()
