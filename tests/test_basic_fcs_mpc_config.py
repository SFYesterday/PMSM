from pathlib import Path

from pmsm.papers.basic_fcs_mpc.config import default_config_path, load_config
from pmsm.papers.basic_fcs_mpc.simulation import run_simulation


def test_default_config_exists_and_loadable():
    path = default_config_path()
    assert path.exists()

    cfg = load_config(path)
    assert cfg.simulation.Ts > 0.0
    assert cfg.motor.Ld > 0.0
    assert cfg.references.iq_ref >= 0.0


def test_run_simulation_uses_default_config_and_overrides():
    cfg = load_config(default_config_path())
    results = run_simulation(iq_ref=cfg.references.iq_ref + 1.0, t_sim=0.01)

    assert len(results["time"]) == int(0.01 / cfg.simulation.Ts)
    assert results["iq_ref_log"][0] == cfg.references.iq_ref + 1.0


def test_run_simulation_with_custom_config_path(tmp_path: Path):
    custom = tmp_path / "custom_fcs.toml"
    custom.write_text(
        """
[simulation]
Ts = 0.0002
T_sim = 0.02
Udc = 180.0
Ir_max = 9.0
load_torque = 1.0

[motor]
Ld = 0.012
Lq = 0.02
Rs = 0.636
psi_pm = 0.088
p = 5
J = 0.001
B_fric = 0.0017

[controller]
lambda_id = 1.0
lambda_iq = 1.0
lambda_sw = 0.0
id_ref_default = 0.0

[references]
iq_ref = 6.0
id_ref = 0.0
""".strip(),
        encoding="utf-8",
    )

    results = run_simulation(config_path=custom)
    assert len(results["time"]) == int(0.02 / 0.0002)
    assert results["iq_ref_log"][0] == 6.0
