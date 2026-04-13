from pmsm.common.motor_model import MotorParams, MotorState, step_motor_state
from pmsm.common.transforms import switch_state_to_dq


def test_switch_state_to_dq_returns_two_values():
    ud, uq = switch_state_to_dq([1, 0, 0], 200.0, 0.0)
    assert isinstance(ud, float)
    assert isinstance(uq, float)


def test_step_motor_state_advances_state():
    params = MotorParams()
    state = MotorState()
    new_state, torque = step_motor_state(state, 0.0, 0.0, 0.0, params)
    assert isinstance(new_state, MotorState)
    assert isinstance(torque, float)
