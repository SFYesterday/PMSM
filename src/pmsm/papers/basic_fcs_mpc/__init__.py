"""基础 FCS-MPC 控制 PMSM 的独立模块。"""

from .config import BasicFCSMPCConfig, default_config_path, load_config
from .controller import FCSMPCController, FCSMPCParams, FCSMPCState
from .simulation import main, run_simulation

__all__ = [
    "BasicFCSMPCConfig",
    "FCSMPCController",
    "FCSMPCParams",
    "FCSMPCState",
    "default_config_path",
    "load_config",
    "main",
    "run_simulation",
]
