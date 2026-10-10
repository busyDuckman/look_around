"""lookaround: natural gaze for anything that can look around.

    import lookaround as la

    looker = la.Looker.from_preset("human_standing")
    looker.report({"base.yaw": 0.1})            # optional: measured pose (closed loop)
    out = looker.update(1 / 60, (1.5, 0.4, 1.2)) # target point in the mount frame, or None
    out.setpoints["head.yaw"].position

Behaviour is defined by docs/SPEC.md; this package is the reference implementation.
"""

import logging

from .looker import Diagnostics, Looker, Output, Setpoint
from .settings import Settings, SettingsError, load_preset, preset_path

__version__ = "0.1.0.dev0"
SPEC_VERSION = "0.1.0"

# Library logging: silent unless the application configures the "lookaround" logger.
logging.getLogger("lookaround").addHandler(logging.NullHandler())

__all__ = [
    "SPEC_VERSION",
    "Diagnostics",
    "Looker",
    "Output",
    "Setpoint",
    "Settings",
    "SettingsError",
    "load_preset",
    "preset_path",
]
