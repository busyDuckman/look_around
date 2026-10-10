"""The head_only preset in open loop must reproduce idea.md (spec section 11).

idea.md's reference code is copied here (degrees), with its 0.3 deg rest offset removed as
spec 8.1 requires.
"""

from __future__ import annotations

import math

import pytest
from conftest import preset_doc

import lookaround as la

REST_PITCH = -8.0
HMCR_YAW, HMCR_UP = 0.69, 0.52
DOWN_KNEE, DOWN_SOFT, DOWN_SHARE = 10.0, 4.0, 1.0
EYE_MAX = 35.0


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _softplus(z: float) -> float:
    return z + math.log1p(math.exp(-z)) if z > 0 else math.log1p(math.exp(z))


def gaze_angles(eye: tuple[float, float, float], point: tuple[float, float, float]):  # type: ignore[no-untyped-def]
    d = [point[i] - eye[i] for i in range(3)]
    yaw = _wrap(math.degrees(math.atan2(d[1], d[0])))
    pitch = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
    return yaw, pitch


def head_share(yaw: float, pitch: float) -> tuple[float, float]:
    hy = max(HMCR_YAW * abs(yaw), abs(yaw) - EYE_MAX)
    e = pitch - REST_PITCH
    if e > 0:
        hp = max(HMCR_UP * e, e - EYE_MAX)
    else:
        z = (-e - DOWN_KNEE) / DOWN_SOFT
        # spec 8.1: subtract the hinge's value at zero
        sp = _softplus(z) - _softplus(-DOWN_KNEE / DOWN_SOFT)
        hp = -max(DOWN_SHARE * DOWN_SOFT * sp, -e - EYE_MAX)
    return math.copysign(hy, yaw), REST_PITCH + hp


class HeadSpring:
    OMEGA = 5.0
    MAX_SPEED = 90.0

    def __init__(self, fps: float) -> None:
        self.dt = 1.0 / fps
        self.yaw, self.pitch = 0.0, REST_PITCH
        self.vyaw = self.vpitch = 0.0

    def step(self, ty: float, tp: float) -> tuple[float, float]:
        tp = max(-35.0, min(30.0, tp))
        w, dt = self.OMEGA, self.dt
        for a, v, tgt in (("yaw", "vyaw", ty), ("pitch", "vpitch", tp)):
            x, vel = getattr(self, a), getattr(self, v)
            err = _wrap(tgt - x) if a == "yaw" else tgt - x
            vel += (w * w * err - 2 * w * vel) * dt
            vel = max(-self.MAX_SPEED, min(self.MAX_SPEED, vel))
            setattr(self, v, vel)
            setattr(self, a, x + vel * dt)
        return self.yaw, self.pitch


def target_at(t: float) -> tuple[float, float, float]:
    # Wanders left/right and up/down in front, with a jump at t = 3 s.
    side = 1.2 * math.sin(0.7 * t) + (1.5 if t > 3.0 else 0.0)
    height = 1.6 + 0.8 * math.sin(0.45 * t) - (0.6 if 5.0 < t < 7.0 else 0.0)
    return (2.0, side, height)


@pytest.mark.parametrize("fps", [30.0, 60.0])
def test_head_only_matches_idea_md(fps: float) -> None:
    doc = preset_doc("head_only")
    doc["max_substep"] = 1.0 / fps  # idea.md integrates one step per frame
    looker = la.Looker(la.Settings.load(doc))
    ref = HeadSpring(fps)
    eye = (0.0, 0.0, 1.6)
    dt = 1.0 / fps
    worst = 0.0
    for k in range(int(10 * fps)):
        t = k * dt
        p = target_at(t)
        hy_ref, hp_ref = ref.step(*head_share(*gaze_angles(eye, p)))
        out = looker.update(dt, p)
        hy = math.degrees(out.setpoints["head.yaw"].position)
        hp = math.degrees(out.setpoints["head.pitch"].position)
        worst = max(worst, abs(_wrap(hy - hy_ref)), abs(hp - hp_ref))
        # the eyes take the remainder exactly while within their range
        if out.diagnostics.reachable:
            assert out.diagnostics.gaze_error < 1e-9
    assert worst < 1e-9
