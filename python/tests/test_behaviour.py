from __future__ import annotations

import copy
import math
from typing import Any

import pytest
from conftest import PRESETS, preset_doc
from hypothesis import given, settings
from hypothesis import strategies as st

import lookaround as la
from lookaround._math import wrap

DT = 1.0 / 60.0
DEG = math.pi / 180.0


def run(looker: la.Looker, target: Any, seconds: float, dt: float = DT) -> la.Output:
    out = None
    for _ in range(int(round(seconds / dt))):
        out = looker.update(dt, target)
    assert out is not None
    return out


def looker(name: str, **changes: Any) -> la.Looker:
    d = copy.deepcopy(preset_doc(name))
    for path, value in changes.items():
        node = d
        keys = path.split("__")
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    return la.Looker(la.Settings.load(d))


# ---- basic tracking --------------------------------------------------------------------------

@pytest.mark.parametrize("name", PRESETS)
def test_settles_on_reachable_target(name: str) -> None:
    lk = looker(name)
    eye_z = 1.2 if name.startswith("human") else 0.4
    out = run(lk, (2.0, 0.5, eye_z), 6.0)
    assert out.diagnostics.reachable
    assert out.diagnostics.gaze_error < 0.2 * DEG


def test_deterministic() -> None:
    a, b = la.Looker.from_preset("human_standing"), la.Looker.from_preset("human_standing")
    for k in range(200):
        p = (1.5, math.sin(k / 30), 1.4)
        assert a.update(DT, p).to_dict() == b.update(DT, p).to_dict()


def test_idle_returns_to_rest() -> None:
    lk = looker("human_standing")
    run(lk, (0.5, 2.0, 1.0), 1.0)
    out = run(lk, None, 6.0)
    sp = out.setpoints
    assert abs(sp["torso.yaw"].position) < 0.5 * DEG
    assert abs(sp["head.yaw"].position) < 0.5 * DEG
    assert sp["head.pitch"].position == pytest.approx(-8 * DEG, abs=0.5 * DEG)
    assert out.diagnostics.gaze_error == 0.0


# ---- the split -------------------------------------------------------------------------------

def test_eyes_do_small_shifts_head_does_share() -> None:
    lk = looker("head_only")
    out = run(lk, (2.0, math.tan(20 * DEG) * 2.0, 1.6 + 2.0 * math.tan(-8 * DEG)), 4.0)
    assert math.degrees(out.setpoints["head.yaw"].position) == pytest.approx(0.69 * 20, abs=0.05)


def test_torso_joins_only_for_large_shifts() -> None:
    lk = looker("human_seated")  # no base, so the torso split is visible in steady state
    out = run(lk, (math.cos(30 * DEG), math.sin(30 * DEG), 1.2), 4.0)
    assert abs(out.setpoints["torso.yaw"].position) < 0.5 * DEG
    lk = looker("human_standing", base__yaw__dynamics=None, base__external=True)
    out = run(lk, (math.cos(100 * DEG), math.sin(100 * DEG), 1.6), 4.0)
    torso = math.degrees(out.setpoints["torso.yaw"].position)
    assert 12.0 < torso < 30.0  # about 20% of a 100 deg shift (Sidenmark & Gellersen 2019)


# ---- base decision ---------------------------------------------------------------------------

def test_base_ignores_a_glance() -> None:
    lk = looker("human_standing")
    out = run(lk, (math.cos(50 * DEG), math.sin(50 * DEG), 1.5), 0.8)
    out = run(lk, (1.0, 0.0, 1.5), 3.0)
    assert out.diagnostics.base_state == "hold"
    assert abs(out.setpoints["base.yaw"].position) < 1 * DEG


def test_base_turns_after_dwell_and_immediately_when_behind() -> None:
    lk = looker("human_standing")
    out = run(lk, (math.cos(50 * DEG), math.sin(50 * DEG), 1.5), 1.0)
    assert out.diagnostics.base_state == "hold"
    out = run(lk, (math.cos(50 * DEG), math.sin(50 * DEG), 1.5), 5.0)
    assert math.degrees(out.setpoints["base.yaw"].position) == pytest.approx(50, abs=3)

    lk = looker("human_standing")
    lk.update(DT, (-1.0, 0.2, 1.5))
    lk.update(DT, (-1.0, 0.2, 1.5))
    out = lk.update(DT, (-1.0, 0.2, 1.5))
    assert out.diagnostics.base_state == "turn"


# ---- latency gate ----------------------------------------------------------------------------

def test_head_starts_after_latency_eyes_immediately() -> None:
    lk = looker("human_seated")  # head latency 0.1 s
    run(lk, (2.0, 0.0, 1.2), 3.0)
    target = (2.0, 1.2, 1.2)
    moved_at = None
    for k in range(30):
        out = lk.update(DT, target)
        if k == 0:
            assert abs(out.setpoints["eyes.yaw"].position) > 10 * DEG  # eyes jump at once
        if moved_at is None and abs(out.setpoints["head.yaw"].velocity) > 1 * DEG:
            moved_at = (k + 1) * DT
    assert moved_at is not None
    assert 0.09 <= moved_at <= 0.15


# ---- invariants (property-based) -------------------------------------------------------------

targets = st.one_of(
    st.none(),
    st.tuples(st.floats(-4, 4), st.floats(-4, 4), st.floats(-1, 3)),
)


@settings(max_examples=60, deadline=None)
@given(
    name=st.sampled_from(PRESETS),
    path=st.lists(st.tuples(targets, st.integers(1, 40)), min_size=1, max_size=6),
    dt=st.sampled_from([1 / 240, 1 / 60, 1 / 30, 1 / 15]),
)
def test_limits_and_speed_never_exceeded(name: str, path: list[Any], dt: float) -> None:
    lk = la.Looker.from_preset(name)
    s = lk.settings
    for target, steps in path:
        for _ in range(steps):
            out = lk.update(dt, target)
            for key, sp in out.setpoints.items():
                assert math.isfinite(sp.position) and math.isfinite(sp.velocity)
                part = getattr(s, key.split(".")[0])
                axis = getattr(part, key.split(".")[-1])
                if axis.limits is not None:
                    assert axis.limits[0] - 1e-12 <= sp.position <= axis.limits[1] + 1e-12
                else:
                    assert -math.pi < sp.position <= math.pi
                if axis.dynamics is not None:
                    assert abs(sp.velocity) <= axis.dynamics.v_max + 1e-9


# ---- closed loop: compensation (spec 8) and the governor (spec 9.3) --------------------------

def test_external_torso_is_compensated() -> None:
    lk = looker("human_standing", torso__external=True, base__external=True)
    assert "torso.yaw" not in lk.outputs
    target = (2.0, 0.6, 1.4)
    for k in range(300):
        twist = 0.35 * math.sin(k * DT * 2.0)  # someone else swivels the torso
        lk.report({"torso.yaw": twist, "base.yaw": 0.0})
        out = lk.update(DT, target)
    assert out.diagnostics.gaze_error < 1.0 * DEG


def test_stuck_internal_joint_is_compensated() -> None:
    # Torso commanded but physically held at 0 (e.g. a balance controller has priority).
    lk = looker("human_standing", base__external=True)
    target = (math.cos(110 * DEG) * 2, math.sin(110 * DEG) * 2, 1.6)
    for _ in range(400):
        lk.report({"torso.yaw": 0.0, "base.yaw": 0.0})
        out = lk.update(DT, target)
    assert out.setpoints["torso.yaw"].position > 5 * DEG  # it keeps asking
    assert out.diagnostics.gaze_error < 1.0 * DEG          # head and eyes made up for it


def test_governor_bounds_lead_under_actuator_lag() -> None:
    lk = looker("pan_tilt")
    max_lead = 15 * DEG
    meas = {"head.yaw": 0.0, "head.pitch": 0.0}
    tau = 0.25  # slow servo: first-order lag
    worst = 0.0
    for k in range(600):
        t = k * DT
        target = (math.cos(t * 2.0) * 2, math.sin(t * 2.0) * 2, 0.1 + math.sin(t))
        lk.report(meas)
        out = lk.update(DT, target)
        for key in meas:
            sp = out.setpoints[key].position
            worst = max(worst, abs(wrap(sp - meas[key])))
            meas[key] = wrap(meas[key] + wrap(sp - meas[key]) * (1 - math.exp(-DT / tau)))
    assert worst <= max_lead + 1e-9


def test_velocity_estimated_when_not_reported() -> None:
    lk = looker("human_standing", torso__external=True, base__external=True)
    for k in range(120):
        lk.report({"torso.yaw": 0.5 * k * DT, "base.yaw": 0.0})
        lk.update(DT, (2.0, 0.0, 1.5))
    assert lk._channels["torso.yaw"].v_meas == pytest.approx(0.5, rel=1e-3)  # pyright: ignore[reportPrivateUsage]


# ---- geometry --------------------------------------------------------------------------------

def test_binocular_vergence() -> None:
    lk = looker("human_seated", eyes__binocular=True)
    assert "eyes.left.yaw" in lk.outputs
    near = run(lk, (0.4, 0.0, 1.2), 3.0).diagnostics.vergence
    far = run(lk, (5.0, 0.0, 1.2), 3.0).diagnostics.vergence
    assert near is not None and far is not None
    assert near > far
    assert far == pytest.approx(2 * math.atan(0.0315 / 4.9), rel=0.15)


def test_long_way_round_for_limited_yaw() -> None:
    d = copy.deepcopy(preset_doc("pan_tilt"))
    d["head"]["yaw"]["limits"] = [-170, 170]
    lk = la.Looker(la.Settings.load(d))
    run(lk, (math.cos(160 * DEG), math.sin(160 * DEG), 0.1), 3.0)
    passed_front = False
    for _ in range(400):
        out = lk.update(DT, (math.cos(-160 * DEG), math.sin(-160 * DEG), 0.1))
        y = out.setpoints["head.yaw"].position
        assert -170 * DEG - 1e-12 <= y <= 170 * DEG + 1e-12
        passed_front |= abs(y) < 5 * DEG
    assert passed_front
    assert math.degrees(out.setpoints["head.yaw"].position) == pytest.approx(-160, abs=1)


def test_wide_limits_pick_nearest_turn() -> None:
    d = copy.deepcopy(preset_doc("pan_tilt"))
    d["head"]["yaw"]["limits"] = [-270, 270]
    lk = la.Looker(la.Settings.load(d))
    run(lk, (math.cos(150 * DEG), math.sin(150 * DEG), 0.1), 3.0)
    out = run(lk, (math.cos(-170 * DEG), math.sin(-170 * DEG), 0.1), 3.0)
    assert math.degrees(out.setpoints["head.yaw"].position) == pytest.approx(190, abs=1)


# ---- logging and input checks ----------------------------------------------------------------

def test_logs_state_changes_once(caplog: pytest.LogCaptureFixture) -> None:
    caplog.set_level("DEBUG", logger="lookaround")
    lk = looker("human_seated")  # no base: a target behind stays out of reach
    run(lk, (-2.0, 0.1, 1.2), 1.0)
    run(lk, (2.0, 0.1, 1.2), 1.0)
    msgs = [r.getMessage() for r in caplog.records]
    assert sum("out of reach" in m for m in msgs) == 1
    assert sum("back within reach" in m for m in msgs) == 1
    assert any("eyes.yaw needs" in m for m in msgs)


def test_logs_base_turn_and_governor(caplog: pytest.LogCaptureFixture) -> None:
    caplog.set_level("DEBUG", logger="lookaround")
    lk = looker("human_standing")
    run(lk, (-1.0, 0.2, 1.5), 4.0)
    msgs = [r.getMessage() for r in caplog.records]
    assert any("base: hold -> turn" in m and "turn_now" in m for m in msgs)
    assert any("base: turn -> hold" in m for m in msgs)

    caplog.clear()
    lk = looker("pan_tilt")
    for _ in range(30):
        lk.report({"head.yaw": 0.0, "head.pitch": 0.0})  # servo not moving at all
        lk.update(DT, (0.0, 2.0, 0.1))
    assert any("head.yaw: plan is" in r.getMessage() for r in caplog.records)


def test_warns_about_unreported_external_joint(caplog: pytest.LogCaptureFixture) -> None:
    lk = looker("human_standing", torso__external=True)
    lk.update(DT, (2.0, 0.0, 1.5))
    warnings = [r for r in caplog.records if r.levelname == "WARNING"]
    assert len(warnings) == 1 and "torso.yaw" in warnings[0].getMessage()


def test_rejects_non_finite_inputs() -> None:
    lk = la.Looker.from_preset("human_standing")
    with pytest.raises(ValueError):
        lk.report({"head.yaw": float("nan")})
    with pytest.raises(ValueError):
        lk.update(DT, (1.0, float("inf"), 1.0))
    with pytest.raises(KeyError):
        lk.report({"neck.yaw": 0.0})
