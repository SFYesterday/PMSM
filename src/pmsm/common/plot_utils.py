"""统一的画图工具（波形、频谱等）。"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np

from .motor_model import MotorParams


def plot_mp_dsc_results(
    time: np.ndarray,
    w_ref_log: np.ndarray,
    w_log: np.ndarray,
    Te_log: np.ndarray,
    Tl_hat_log: np.ndarray,
    id_log: np.ndarray,
    iq_log: np.ndarray,
    motor_params: MotorParams,
) -> None:
    plt.figure(figsize=(10, 12), facecolor="white")

    plt.subplot(4, 1, 1)
    plt.plot(time, w_ref_log * (60 / (2 * math.pi)), "r--", linewidth=1.5, label="Reference")
    plt.plot(time, w_log, "b", linewidth=1.5, label="Actual")
    plt.ylabel("Speed (rpm)")
    plt.title("Speed with Load Torque Rejection (t=0.2s added 6Nm)")
    plt.legend(loc="lower right")
    plt.grid(True)

    plt.subplot(4, 1, 2)
    plt.plot(time, Te_log, "k", linewidth=1.0, label="Electromagnetic Te")
    plt.plot(time, Tl_hat_log, "m--", linewidth=1.5, label="Observed Load Tl_hat")
    plt.ylabel("Torque (Nm)")
    plt.legend(loc="lower right")
    plt.grid(True)

    plt.subplot(4, 1, 3)
    plt.plot(time, id_log, "b", linewidth=1.0, label="i_d (MTPA track)")
    plt.plot(time, iq_log, "g", linewidth=1.0, label="i_q")
    plt.ylabel("Current (A)")
    plt.legend(loc="lower right")
    plt.grid(True)

    plt.subplot(4, 1, 4)
    plt.plot(id_log, iq_log, "b", linewidth=1.0, label="Actual Trajectory")

    id_mtpa = np.linspace(-10, 0, 100)
    iq_mtpa = np.sqrt(np.clip((id_mtpa) / ((motor_params.Lq - motor_params.Ld) / motor_params.psi_pm) + id_mtpa**2, 0.0, None))
    plt.plot(id_mtpa, iq_mtpa, "r--", linewidth=2, label="Theoretical MTPA")

    plt.xlabel("i_d (A)")
    plt.ylabel("i_q (A)")
    plt.title("Current Trajectory & MTPA Curve")
    plt.legend(loc="upper right")
    plt.grid(True)
    plt.xlim([-10, 2])
    plt.ylim([0, 12])

    plt.tight_layout()
    plt.show()
