from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from pmsm.common.plot_utils import plot_basic_fcs_mpc_results
from pmsm.papers.basic_fcs_mpc.simulation import run_simulation


def test_basic_fcs_mpc_plot_can_be_saved(tmp_path: Path):
    results = run_simulation(t_sim=0.01)
    target = tmp_path / "basic_fcs_mpc.png"

    fig = plot_basic_fcs_mpc_results(
        results["time"],
        results["id_ref_log"],
        results["iq_ref_log"],
        results["id_log"],
        results["iq_log"],
        results["w_log"],
        results["Te_log"],
        results["switch_idx_log"],
        save_path=target,
        show=False,
    )

    assert fig is not None
    assert target.exists()
