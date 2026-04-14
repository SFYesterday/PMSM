"""basic_fcs_mpc 结构化配置加载。"""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
import tomllib


@dataclass(frozen=True)
class SimulationConfig:
    Ts: float = 100e-6
    T_sim: float = 0.2
    Udc: float = 200.0
    Ir_max: float = 10.0
    load_torque: float = 2.0


@dataclass(frozen=True)
class MotorConfig:
    Ld: float = 12e-3
    Lq: float = 20e-3
    Rs: float = 0.636
    psi_pm: float = 0.088
    p: int = 5
    J: float = 1.0e-3
    B_fric: float = 1.7e-3


@dataclass(frozen=True)
class ControllerConfig:
    lambda_id: float = 1.0
    lambda_iq: float = 1.0
    lambda_sw: float = 0.0
    id_ref_default: float = 0.0


@dataclass(frozen=True)
class ReferenceConfig:
    iq_ref: float = 8.0
    id_ref: float = 0.0


@dataclass(frozen=True)
class BasicFCSMPCConfig:
    simulation: SimulationConfig = SimulationConfig()
    motor: MotorConfig = MotorConfig()
    controller: ControllerConfig = ControllerConfig()
    references: ReferenceConfig = ReferenceConfig()


def default_config_path() -> Path:
    """返回基础 FCS-MPC 默认配置路径。"""

    repo_root = Path(__file__).resolve().parents[4]
    return repo_root / "configs" / "basic_fcs_mpc" / "fcs_mpc.toml"


def _section_from_dict(section_cls: type, data: dict[str, object] | None):
    if data is None:
        return section_cls()
    valid_keys = {f.name for f in fields(section_cls)}
    filtered = {k: v for k, v in data.items() if k in valid_keys}
    return section_cls(**filtered)


def load_config(config_path: str | Path | None = None) -> BasicFCSMPCConfig:
    """读取 TOML 并返回结构化配置对象。"""

    path = Path(config_path) if config_path is not None else default_config_path()
    if not path.exists():
        raise FileNotFoundError(f"basic_fcs_mpc config not found: {path}")

    with path.open("rb") as f:
        raw = tomllib.load(f)

    simulation = _section_from_dict(SimulationConfig, raw.get("simulation"))
    motor = _section_from_dict(MotorConfig, raw.get("motor"))
    controller = _section_from_dict(ControllerConfig, raw.get("controller"))
    references = _section_from_dict(ReferenceConfig, raw.get("references"))
    return BasicFCSMPCConfig(
        simulation=simulation,
        motor=motor,
        controller=controller,
        references=references,
    )
