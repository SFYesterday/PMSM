"""统一的绘图工具。"""

from __future__ import annotations

import math
from pathlib import Path

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
    save_path: str | Path | None = None,
    show: bool = True,
):
    fig = plt.figure(figsize=(10, 12), facecolor="white")

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

    fig.tight_layout()

    if save_path is not None:
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=200, bbox_inches="tight")

    if show:
        plt.show()

    return fig


def plot_basic_fcs_mpc_results(
    time: np.ndarray,
    id_ref_log: np.ndarray,
    iq_ref_log: np.ndarray,
    id_log: np.ndarray,
    iq_log: np.ndarray,
    w_log: np.ndarray,
    te_log: np.ndarray,
    switch_idx_log: np.ndarray,
    save_path: str | Path | None = None,
    show: bool = True,
):
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), facecolor="white", sharex=True)

    axes[0].plot(time, id_ref_log, "r--", linewidth=1.2, label="i_d*")
    axes[0].plot(time, id_log, "b", linewidth=1.2, label="i_d")
    axes[0].plot(time, iq_ref_log, "m--", linewidth=1.2, label="i_q*")
    axes[0].plot(time, iq_log, "g", linewidth=1.2, label="i_q")
    axes[0].set_ylabel("Current (A)")
    axes[0].set_title("Basic FCS-MPC Current Tracking")
    axes[0].legend(loc="best")
    axes[0].grid(True)

    axes[1].plot(time, iq_ref_log - iq_log, "k", linewidth=1.1, label="i_q error")
    axes[1].plot(time, id_ref_log - id_log, "c", linewidth=1.1, label="i_d error")
    axes[1].set_ylabel("Error (A)")
    axes[1].legend(loc="best")
    axes[1].grid(True)

    axes[2].plot(time, te_log, color="tab:orange", linewidth=1.2, label="Electromagnetic torque")
    axes[2].plot(time, w_log, color="tab:blue", linewidth=1.2, label="Mechanical speed")
    axes[2].set_ylabel("Torque / rpm")
    axes[2].legend(loc="best")
    axes[2].grid(True)

    axes[3].step(time, switch_idx_log, where="post", color="tab:purple", linewidth=1.0, label="Switch index")
    axes[3].set_xlabel("Time (s)")
    axes[3].set_ylabel("State")
    axes[3].set_yticks(range(8))
    axes[3].legend(loc="best")
    axes[3].grid(True)

    fig.tight_layout()

    if save_path is not None:
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=200, bbox_inches="tight")

    if show:
        plt.show()

    return fig
