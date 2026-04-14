import numpy as np

from pmsm.papers.basic_fcs_mpc.simulation import run_simulation


def _steady_state_mae(reference: np.ndarray, signal: np.ndarray, tail_ratio: float = 0.2) -> float:
    start = int(len(signal) * (1.0 - tail_ratio))
    return float(np.mean(np.abs(reference[start:] - signal[start:])))


def test_basic_fcs_mpc_tracks_iq_well_in_low_speed_region():
    results = run_simulation(load_torque=4.0, t_sim=0.1)

    iq_ss_mae = _steady_state_mae(results["iq_ref_log"], results["iq_log"])
    id_ss_mae = _steady_state_mae(results["id_ref_log"], results["id_log"])

    assert iq_ss_mae < 0.3
    assert id_ss_mae < 0.3
    assert abs(results["iq_log"][-1] - results["iq_ref_log"][-1]) < 0.3


def test_basic_fcs_mpc_default_case_exposes_high_speed_tracking_limit():
    results = run_simulation()

    iq_ss_mae = _steady_state_mae(results["iq_ref_log"], results["iq_log"])

    assert results["w_log"][-1] > 1500.0
    assert iq_ss_mae > 3.0
    assert results["iq_log"][-1] < results["iq_ref_log"][-1] - 3.0
