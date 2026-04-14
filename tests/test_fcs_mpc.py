import numpy as np

from pmsm.common.motor_model import MotorParams, MotorState, step_motor_state
from pmsm.papers.basic_fcs_mpc.controller import FCSMPCController


def test_fcs_mpc_step_returns_valid_values():
    controller = FCSMPCController(motor_params=MotorParams())
    state = MotorState()

    ud, uq, idx = controller.step(state, iq_ref=5.0, id_ref=0.0)

    assert np.isfinite(ud)
    assert np.isfinite(uq)
    assert idx in range(8)


def test_fcs_mpc_pushes_iq_positive_from_zero():
    params = MotorParams()
    controller = FCSMPCController(motor_params=params)
    state = MotorState()

    ud, uq, _ = controller.step(state, iq_ref=5.0, id_ref=0.0)
    new_state, _ = step_motor_state(state, ud, uq, load_torque=0.0, params=params)

    assert new_state.iq > 0.0


def test_fcs_mpc_updates_last_switch_index():
    controller = FCSMPCController(motor_params=MotorParams())
    state = MotorState()

    _, _, idx = controller.step(state, iq_ref=3.0, id_ref=0.0)

    assert controller.state.last_state_idx == idx
