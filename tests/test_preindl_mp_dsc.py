import numpy as np

from pmsm.papers.preindl_2013_mp_dsc.simulation import run_simulation


def test_mp_dsc_tracks_500rpm_and_rejects_load_step():
    results = run_simulation()

    time = results["time"]
    speed_rpm = results["w_log"]
    ref_rpm = results["w_ref_log"] * 60.0 / (2.0 * np.pi)
    tl_hat = results["Tl_hat_log"]

    before_load = (time >= 0.15) & (time < 0.2)
    after_load = (time >= 0.2) & (time < 0.3)

    assert abs(np.mean(speed_rpm[before_load]) - 500.0) < 20.0
    assert np.min(speed_rpm[after_load]) > 400.0
    assert abs(speed_rpm[-1] - ref_rpm[-1]) < 20.0
    assert abs(tl_hat[-1] - 6.0) < 0.5


def test_mp_dsc_currents_stay_within_reasonable_bounds():
    results = run_simulation()

    current_mag = np.hypot(results["id_log"], results["iq_log"])

    assert np.max(current_mag) < 12.0
