"""
Turret FCS - Autonomous Air Defense Fire Control System Package
YTU Senior Design Capstone Project
"""

from .config import CONFIG, FCSState

__version__ = "3.0.0"

_LAZY_IMPORTS = {
    "FCSManager": ".fcs_manager",
    "FCSCommunicator": ".fcs_comm",
    "CameraThread": ".camera_thread",
    "TurretHUD": ".fcs_hud",
    "TelemetryLogger": ".telemetry",
    "EngagementManager": ".engagement",
    "EngagementState": ".engagement",
    "HUDPresenter": ".presenter",
    "process_key_event": ".keymap",
    "PController": ".p_controller",
    "RLController": ".rl_controller",
    "KalmanFilter": ".kalman_filter",
    "LockManager": ".lock_manager",
    "TargetSelector": ".target_selector",
    "ensure_weight_file": ".weights",
}

__all__ = [
    "CONFIG",
    "FCSState",
    *_LAZY_IMPORTS.keys(),
]


def __getattr__(name: str):
    if name in _LAZY_IMPORTS:
        import importlib
        module = importlib.import_module(_LAZY_IMPORTS[name], __package__)
        return getattr(module, name)
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")


def __dir__():
    return sorted(__all__)
