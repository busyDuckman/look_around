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
    """Preset with edits, e.g. looker("human_standing", torso__0__external=True)."""
    d = copy.deepcopy(preset_doc(name))
    for path, value in changes.items():
        node: Any = d
        keys = [int(k) if k.isdigit() else k for k in path.split("__")]
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    return la.Looker(la.Settings.load(d))


def polar(deg_az: float, r: float = 2.0, z: float = 1.6) -> tuple[float, float, float]:
    return (r * math.cos(deg_az * DEG), r * math.sin(deg_az * DEG), z)


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
    for key in ("torso.0.yaw", "torso.1.yaw", "head.yaw"):
        assert abs(sp[key].position) < 0.5 * DEG
    assert sp["head.pitch"].position == pytest.approx(-8 * DEG, abs=0.5 * DEG)
    assert out.diagnostics.gaze_error == 0.0


# ---- world frame and root (spec 3) ------------------------------------------------------------

def test_target_is_a_world_point_relative_to_the_root() -> None:
    lk = looker("human_seated")
    lk.set_root(position=(5.0, 5.0, 0.0), heading=0.0)
    out = run(lk, (7.0, 5.0, 1.2), 3.0)       # straight ahead of where it stands
    assert abs(out.setpoints["head.yaw"].position) < 0.5 * DEG
    assert abs(out.setpoints["eyes.yaw"].position) < 0.5 * DEG
    lk.set_root(heading=math.pi / 2)          # now facing world +y: the target is to its right
    out = run(lk, (7.0, 5.0, 1.2), 3.0)
    assert out.setpoints["head.yaw"].position < -40 * DEG


@pytest.mark.parametrize("name", ["human_standing", "human_seated", "pan_tilt"])
def test_frame_invariance(name: str) -> None:
    # Moving and rotating the whole scene must not change any joint, only the heading.
    shift, turn = (3.0, -2.0, 0.0), 0.7
    c, s = math.cos(turn), math.sin(turn)
    a, b = looker(name), looker(name)
    a.set_root(position=(0.0, 0.0, 0.0), heading=0.0)
    b.set_root(position=shift, heading=turn)
    for k in range(300):
        t = k * DT
        p = (2.0 * math.cos(0.8 * t), 2.0 * math.sin(0.8 * t) + 0.5, 1.3 + 0.3 * math.sin(t))
        q = (c * p[0] - s * p[1] + shift[0], s * p[0] + c * p[1] + shift[1], p[2] + shift[2])
        oa, ob = a.update(DT, p), b.update(DT, q)
        for key, sp in oa.setpoints.items():
            other = ob.setpoints[key].position
            if key == "heading.yaw":
                assert wrap(other - sp.position - turn) == pytest.approx(0.0, abs=1e-9)
            else:
                assert other == pytest.approx(sp.position, abs=1e-9)


# ---- the split -------------------------------------------------------------------------------

def test_eyes_do_small_shifts_head_does_share() -> None:
    lk = looker("head_only")  # no dead zone: Kim et al. share from the first degree
    out = run(lk, (2.0, math.tan(20 * DEG) * 2.0, 1.6 + 2.0 * math.tan(-8 * DEG)), 4.0)
    assert math.degrees(out.setpoints["head.yaw"].position) == pytest.approx(0.69 * 20, abs=0.05)


def test_head_dead_zone() -> None:
    lk = looker("human_seated")
    out = run(lk, polar(8, z=1.2 + 2 * math.tan(-8 * DEG)), 4.0)   # 8 deg to the side
    # Eyes do almost all of it. The 3 deg softness lets ~0.5 deg leak in at 8 deg (spec 4.4),
    # against 0.69 * 8 = 5.5 deg with no dead zone.
    assert abs(out.setpoints["head.yaw"].position) < 0.8 * DEG
    assert math.degrees(out.setpoints["eyes.yaw"].position) == pytest.approx(8, abs=1.0)
    out = run(lk, polar(40, z=1.2 + 2 * math.tan(-8 * DEG)), 4.0)
    assert out.setpoints["head.yaw"].position > 15 * DEG            # head joins beyond it


def test_torso_chain_joins_only_for_large_shifts() -> None:
    lk = looker("human_standing", heading__external=True)
    lk.set_root(heading=0.0)
    out = run(lk, polar(30), 4.0)
    assert abs(out.setpoints["torso.0.yaw"].position) < 0.5 * DEG
    assert abs(out.setpoints["torso.1.yaw"].position) < 0.5 * DEG
    out = run(lk, polar(100), 4.0)
    pelvis = math.degrees(out.setpoints["torso.0.yaw"].position)
    chest = math.degrees(out.setpoints["torso.1.yaw"].position)
    # about 20% of a 100 deg shift (Sidenmark & Gellersen 2019)
    assert 12.0 < pelvis + chest < 30.0
    assert 0.6 < pelvis / chest < 1.6     # pelvis takes about half (Pejsa 2015)


# ---- heading decision (spec 8.3) ---------------------------------------------------------------

def test_heading_ignores_a_glance() -> None:
    lk = looker("human_standing")
    run(lk, polar(50, z=1.5), 0.8)
    out = run(lk, (1.0, 0.0, 1.5), 3.0)
    assert out.diagnostics.heading_state == "hold"
    assert abs(out.setpoints["heading.yaw"].position) < 1 * DEG


def test_heading_turns_after_dwell_and_immediately_when_behind() -> None:
    lk = looker("human_standing")
    out = run(lk, polar(50, z=1.5), 1.0)
    assert out.diagnostics.heading_state == "hold"
    out = run(lk, polar(50, z=1.5), 5.0)
    assert math.degrees(out.setpoints["heading.yaw"].position) == pytest.approx(50, abs=3)

    lk = looker("human_standing")
    for _ in range(3):
        out = lk.update(DT, (-1.0, 0.2, 1.5))
    assert out.diagnostics.heading_state == "turn"


def test_heading_fed_back_through_set_root() -> None:
    # Internal heading, but the robot cannot actually turn: head and torso make up the rest.
    lk = looker("human_standing")
    for _ in range(400):
        lk.set_root(heading=0.0)
        out = lk.update(DT, polar(120))
    assert out.setpoints["heading.yaw"].position > 60 * DEG   # it keeps asking to turn
    assert out.diagnostics.gaze_error < 1.0 * DEG
    with pytest.raises(KeyError):
        lk.feedback({"heading.yaw": 0.0})


# ---- reachable query (spec 7) ------------------------------------------------------------------

def test_reachable_query() -> None:
    lk = looker("human_seated")
    assert lk.reachable((2.0, 0.3, 1.2))
    assert not lk.reachable((-2.0, 0.1, 1.2))       # behind; the seated looker cannot turn
    assert not lk.reachable((0.3, 0.0, 6.0))        # straight overhead: beyond head + eyes pitch
    before = run(looker("human_seated"), (2.0, 0.3, 1.2), 1.0).to_dict()
    lk2 = looker("human_seated")
    lk2.reachable((-2.0, 0.1, 1.2))                 # a query changes nothing
    assert run(lk2, (2.0, 0.3, 1.2), 1.0).to_dict() == before
    assert looker("pan_tilt").reachable((-2.0, 0.0, 0.5))   # continuous pan


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
    parts = {p.key: p for p in lk.settings.parts()}
    for target, steps in path:
        for _ in range(steps):
            out = lk.update(dt, target)
            for key, sp in out.setpoints.items():
                assert math.isfinite(sp.position) and math.isfinite(sp.velocity)
                prefix, axis_name = key.rsplit(".", 1)
                part = parts["eyes" if prefix.startswith("eyes") else prefix]
                axis = part.axes()[axis_name]
                if axis.limits is not None:
                    assert axis.limits[0] - 1e-12 <= sp.position <= axis.limits[1] + 1e-12
                else:
                    assert -math.pi < sp.position <= math.pi
                if axis.dynamics is not None:
                    assert abs(sp.velocity) <= axis.dynamics.v_max + 1e-9


# ---- closed loop: compensation (spec 8) and the governor (spec 9.3) --------------------------

def test_external_torso_is_compensated() -> None:
    lk = looker("human_standing", torso__0__external=True, torso__1__external=True,
                heading__external=True)
    assert "torso.0.yaw" not in lk.outputs and "heading.yaw" not in lk.outputs
    target = (2.0, 0.6, 1.4)
    for k in range(300):
        twist = 0.2 * math.sin(k * DT * 2.0)  # someone else swivels the torso
        lk.set_root(heading=0.0)
        lk.feedback({"torso.0.yaw": twist, "torso.1.yaw": twist})
        out = lk.update(DT, target)
    assert out.diagnostics.gaze_error < 1.0 * DEG


def test_stuck_internal_joint_is_compensated() -> None:
    # Torso commanded but physically held at 0 (e.g. a balance controller has priority).
    lk = looker("human_standing", heading__external=True)
    for _ in range(400):
        lk.set_root(heading=0.0)
        lk.feedback({"torso.0.yaw": 0.0, "torso.1.yaw": 0.0})
        out = lk.update(DT, polar(110))
    assert out.setpoints["torso.0.yaw"].position > 3 * DEG   # it keeps asking
    assert out.diagnostics.gaze_error < 1.0 * DEG            # head and eyes made up for it


def test_governor_bounds_lead_under_actuator_lag() -> None:
    lk = looker("pan_tilt")
    max_lead = 15 * DEG
    meas = {"head.yaw": 0.0, "head.pitch": 0.0}
    tau = 0.25  # slow servo: first-order lag
    worst = 0.0
    for k in range(600):
        t = k * DT
        target = (math.cos(t * 2.0) * 2, math.sin(t * 2.0) * 2, 0.1 + math.sin(t))
        lk.feedback(meas)
        out = lk.update(DT, target)
        for key in meas:
            sp = out.setpoints[key].position
            worst = max(worst, abs(wrap(sp - meas[key])))
            meas[key] = wrap(meas[key] + wrap(sp - meas[key]) * (1 - math.exp(-DT / tau)))
    assert worst <= max_lead + 1e-9


def test_velocity_estimated_when_not_fed_back() -> None:
    lk = looker("human_standing", torso__0__external=True, heading__external=True)
    for k in range(120):
        lk.set_root(heading=0.0)
        lk.feedback({"torso.0.yaw": 0.3 * k * DT})
        lk.update(DT, (2.0, 0.0, 1.5))
    assert lk._channels["torso.0.yaw"].v_meas == pytest.approx(0.3, rel=1e-3)  # pyright: ignore[reportPrivateUsage]


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
    lk = looker("pan_tilt", head__yaw__limits=[-170, 170])
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
    lk = looker("pan_tilt", head__yaw__limits=[-270, 270])
    run(lk, (math.cos(150 * DEG), math.sin(150 * DEG), 0.1), 3.0)
    out = run(lk, (math.cos(-170 * DEG), math.sin(-170 * DEG), 0.1), 3.0)
    assert math.degrees(out.setpoints["head.yaw"].position) == pytest.approx(190, abs=1)


# ---- logging and input checks ----------------------------------------------------------------

def test_logs_state_changes_once(caplog: pytest.LogCaptureFixture) -> None:
    caplog.set_level("DEBUG", logger="lookaround")
    lk = looker("human_seated")  # no heading: a target behind stays out of reach
    run(lk, (-2.0, 0.1, 1.2), 1.0)
    run(lk, (2.0, 0.1, 1.2), 1.0)
    msgs = [r.getMessage() for r in caplog.records]
    assert sum("out of reach" in m for m in msgs) == 1
    assert sum("back within reach" in m for m in msgs) == 1
    assert any("eyes.yaw needs" in m for m in msgs)


def test_logs_heading_turn_and_governor(caplog: pytest.LogCaptureFixture) -> None:
    caplog.set_level("DEBUG", logger="lookaround")
    lk = looker("human_standing")
    run(lk, (-1.0, 0.2, 1.5), 4.0)
    msgs = [r.getMessage() for r in caplog.records]
    assert any("heading: hold -> turn" in m and "turn_now" in m for m in msgs)
    assert any("heading: turn -> hold" in m for m in msgs)

    caplog.clear()
    lk = looker("pan_tilt")
    for _ in range(30):
        lk.feedback({"head.yaw": 0.0, "head.pitch": 0.0})  # servo not moving at all
        lk.update(DT, (0.0, 2.0, 0.1))
    assert any("head.yaw: plan is" in r.getMessage() for r in caplog.records)


def test_warns_about_unreported_external_joint(caplog: pytest.LogCaptureFixture) -> None:
    lk = looker("human_standing", torso__1__external=True)
    lk.update(DT, (2.0, 0.0, 1.5))
    warnings = [r for r in caplog.records if r.levelname == "WARNING"]
    assert len(warnings) == 1 and "torso.1.yaw" in warnings[0].getMessage()


def test_rejects_non_finite_inputs() -> None:
    lk = la.Looker.from_preset("human_standing")
    with pytest.raises(ValueError):
        lk.feedback({"head.yaw": float("nan")})
    with pytest.raises(ValueError):
        lk.update(DT, (1.0, float("inf"), 1.0))
    with pytest.raises(ValueError):
        lk.set_root(position=(0.0, float("nan"), 0.0))
    with pytest.raises(KeyError):
        lk.feedback({"neck.yaw": 0.0})
