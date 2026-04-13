"""Clarke/Park 坐标变换工具。"""

from __future__ import annotations

import math

import numpy as np


def clarke_transform(state: np.ndarray | list[int], u_dc: float) -> tuple[float, float]:
    sa, sb, sc = state
    v_alpha = (2.0 / 3.0) * u_dc * (sa - 0.5 * sb - 0.5 * sc)
    v_beta = (2.0 / 3.0) * u_dc * (math.sqrt(3.0) / 2.0 * sb - math.sqrt(3.0) / 2.0 * sc)
    return v_alpha, v_beta


def park_transform(v_alpha: float, v_beta: float, theta_e: float) -> tuple[float, float]:
    cos_t = math.cos(theta_e)
    sin_t = math.sin(theta_e)
    u_d = v_alpha * cos_t + v_beta * sin_t
    u_q = -v_alpha * sin_t + v_beta * cos_t
    return u_d, u_q


def switch_state_to_dq(state: np.ndarray | list[int], u_dc: float, theta_e: float) -> tuple[float, float]:
    v_alpha, v_beta = clarke_transform(state, u_dc)
    return park_transform(v_alpha, v_beta, theta_e)
