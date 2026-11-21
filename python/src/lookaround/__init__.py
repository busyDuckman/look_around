"""lookaround: natural gaze for anything that can look around.

    import lookaround as la

    looker = la.Looker.from_preset("human_standing")
    looker.set_root(position=(2.0, 1.0, 0.0), heading=0.5)   # where it stands in the world
    looker.feedback({"torso.0.yaw": 0.1})                     # optional: measured joints
    out = looker.update(1 / 60, (3.5, 2.0, 1.4))              # target: a world point, or None
    out.setpoints["head.yaw"].position

Behaviour is defined by docs/spec.md; this package is the reference implementation.
"""

import logging

from .looker import Diagnostics, Looker, Output, Setpoint
from .settings import Settings, SettingsError, load_preset, preset_path

__version__ = "0.1.0.dev0"
SPEC_VERSION = "0.2.0"

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
