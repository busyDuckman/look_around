"""The Looker: report the measured pose, update, read setpoints (spec sections 5 to 10)."""

from __future__ import annotations

import logging
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ._math import (
    TAU,
    Frame,
    Vec3,
    add,
    angle_between,
    clamp,
    direction_angles,
    joint_matrix,
    mat_vec,
    norm,
    rpy_matrix,
    sign,
    softplus,
    sub,
    wrap,
)
from .settings import Axis, Hinge, Part, Settings, load_preset

Measurement = float | Sequence[float] | Mapping[str, float]

MIN_TARGET_DISTANCE = 1e-6

log = logging.getLogger("lookaround")


def _deg(rad: float) -> float:
    return math.degrees(rad)
SUBSTEP_EPS = 1e-9


@dataclass(frozen=True)
class Setpoint:
    position: float
    velocity: float
    acceleration: float


@dataclass(frozen=True)
class Diagnostics:
    gaze_error: float
    reachable: bool
    lead: dict[str, float]
    base_state: str | None = None
    vergence: float | None = None


@dataclass(frozen=True)
class Output:
    t: float
    setpoints: dict[str, Setpoint]
    diagnostics: Diagnostics
    pose: dict[str, float] = field(default_factory=dict[str, float])

    def to_dict(self) -> dict[str, Any]:
        d = self.diagnostics
        return {
            "t": self.t,
            "setpoints": {
                k: {"position": s.position, "velocity": s.velocity, "acceleration": s.acceleration}
                for k, s in self.setpoints.items()
            },
            "diagnostics": {
                "gaze_error": d.gaze_error,
                "reachable": d.reachable,
                "lead": dict(d.lead),
                **({"base_state": d.base_state} if d.base_state is not None else {}),
                **({"vergence": d.vergence} if d.vergence is not None else {}),
            },
        }


class _Channel:
    """One joint axis: reference state, latency gate and measurement bookkeeping."""

    def __init__(self, key: str, part: Part, axis_name: str, axis: Axis) -> None:
        self.key = key
        self.part = part
        self.axis_name = axis_name
        self.axis = axis
        self.internal = not part.external
        # reference state
        self.x = axis.rest
        self.v = 0.0
        self.a = 0.0
        self.target = axis.rest
        self.engaged = False
        self.timer = 0.0
        # measurements
        self.reported: tuple[float, float | None] | None = None
        self.last_ext: tuple[float, float] = (axis.rest, 0.0)
        self.prev_meas: float | None = None
        self.v_est = 0.0
        self.x_meas = axis.rest
        self.v_meas = 0.0
        self.lead = 0.0
        self.governed = False      # governor currently holding the plan back
        self.ever_reported = False

    def diff(self, a: float, b: float) -> float:
        return wrap(a - b) if self.axis.continuous else a - b

    def norm_angle(self, a: float) -> float:
        return wrap(a) if self.axis.continuous else a


class Looker:
    """A configured looker. Call report() (optional) then update() once per tick."""

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.t = 0.0
        self._first = True
        self._channels: dict[str, _Channel] = {}
        for part in settings.parts():
            for axis_name, axis in part.axes().items():
                if part.name == "eyes" and part.binocular:
                    for side in ("left", "right"):
                        key = f"eyes.{side}.{axis_name}"
                        self._channels[key] = _Channel(key, part, axis_name, axis)
                else:
                    key = f"{part.name}.{axis_name}"
                    self._channels[key] = _Channel(key, part, axis_name, axis)
        self._reach = {
            (p.name, ax): self._child_reach(p, ax) for p in settings.parts() for ax in p.axes()
        }
        self._base_state = "hold"
        self._held = settings.base.yaw.rest if settings.base else 0.0
        self._dwell = 0.0
        self._was_reachable = True
        self._was_too_close = False
        log.debug("looker %r: parts %s; driving %s; external %s", settings.name,
                  [p.name for p in settings.parts()], self.outputs,
                  [k for k, c in self._channels.items() if not c.internal])

    # ---- construction helpers -------------------------------------------------------------

    @classmethod
    def from_preset(cls, name: str) -> Looker:
        return cls(load_preset(name))

    @classmethod
    def from_file(cls, path: str | Path) -> Looker:
        return cls(Settings.load(path))

    @property
    def channels(self) -> list[str]:
        """Every joint axis key, e.g. 'head.yaw' or 'eyes.left.pitch'."""
        return list(self._channels)

    @property
    def outputs(self) -> list[str]:
        """Keys that update() returns setpoints for (internal parts only)."""
        return [k for k, c in self._channels.items() if c.internal]

    def _child_reach(self, part: Part, axis_name: str) -> tuple[float, float]:
        parts = self.settings.parts()
        pos = neg = 0.0
        for p in parts[parts.index(part) + 1:]:
            ax = p.axes().get(axis_name)
            if ax is None:
                continue
            if ax.limits is None:
                return math.inf, math.inf
            pos += ax.limits[1] - ax.rest
            neg += ax.rest - ax.limits[0]
        return pos, neg

    # ---- public API -----------------------------------------------------------------------

    def report(self, measured: Mapping[str, Measurement]) -> None:
        """Measured positions (and optionally velocities) for the next update only."""
        for key, value in measured.items():
            ch = self._channels.get(key)
            if ch is None:
                raise KeyError(f"unknown joint {key!r}; expected one of {self.channels}")
            if isinstance(value, Mapping):
                pos = float(value["position"])
                vel = value.get("velocity")
                reported = (pos, None if vel is None else float(vel))
            elif isinstance(value, Sequence):
                reported = (float(value[0]), float(value[1]) if len(value) > 1 else None)
            else:
                reported = (float(value), None)
            if not all(math.isfinite(v) for v in reported if v is not None):
                raise ValueError(f"non-finite measurement for {key!r}: {value!r}")
            ch.reported = reported
            ch.ever_reported = True

    def update(self, dt: float, target: Sequence[float] | None) -> Output:
        if not dt > 0:
            raise ValueError("dt must be > 0")
        tgt: Vec3 | None = None
        if target is not None:
            tgt = (float(target[0]), float(target[1]), float(target[2]))
            if not all(math.isfinite(c) for c in tgt):
                raise ValueError(f"non-finite target {target!r}")

        if self._first:
            self._initialise()
        self._resolve_measurements(dt)

        # pose seen by the cascade: predicted measured pose, overwritten part by part (spec 8)
        pose = {k: self._predict(c, c.x_meas, c.v_meas) for k, c in self._channels.items()}

        if tgt is not None:
            frames = self._frames(pose)
            too_close = norm(sub(tgt, self._eye_point(frames))) < MIN_TARGET_DISTANCE
            if too_close != self._was_too_close:
                if too_close:
                    log.debug("target is at the eye point; treating it as idle")
                self._was_too_close = too_close
            if too_close:
                tgt = None

        reachable = True
        out_of_reach: tuple[str, float, tuple[float, float]] | None = None
        for part in self.settings.parts():
            raw, pre_clamp = self._raw_targets(part, pose, tgt, dt)
            innermost = part is self.settings.parts()[-1]
            for key, value in raw.items():
                ch = self._channels[key]
                if innermost and tgt is not None and ch.axis.limits is not None:
                    lo, hi = ch.axis.limits
                    if not lo - 1e-12 <= pre_clamp[key] <= hi + 1e-12:
                        reachable = False
                        out_of_reach = out_of_reach or (key, pre_clamp[key], (lo, hi))
                if ch.internal:
                    self._step(ch, value, dt)
                    if ch.reported is None:
                        pose[key] = self._predict(ch, ch.x, ch.v)

        if tgt is not None and reachable != self._was_reachable:
            if out_of_reach is not None:
                key, need, (lo, hi) = out_of_reach
                log.debug("target out of reach: %s needs %.1f deg, limits [%.1f, %.1f] deg",
                          key, _deg(need), _deg(lo), _deg(hi))
            else:
                log.debug("target back within reach")
            self._was_reachable = reachable
        out = self._output(pose, tgt, reachable)
        for ch in self._channels.values():
            ch.prev_meas = ch.x_meas
            ch.reported = None
        self.t += dt
        return out

    # ---- measurements (spec 6) --------------------------------------------------------------

    def _initialise(self) -> None:
        for ch in self._channels.values():
            if ch.reported is not None:
                ch.x = ch.norm_angle(ch.reported[0])
            ch.v = ch.a = 0.0
            ch.target = ch.x
            if not ch.internal and ch.reported is not None:
                ch.last_ext = (ch.x, 0.0)
        if self.settings.base is not None:
            self._held = self._channels["base.yaw"].x
        for key, ch in self._channels.items():
            if not ch.internal and not ch.ever_reported:
                log.warning("external joint %s was not reported before the first update; "
                            "assuming its rest angle (%.1f deg) until it is", key,
                            _deg(ch.axis.rest))
        self._first = False

    def _resolve_measurements(self, dt: float) -> None:
        tau = self.settings.velocity_filter_tau
        for ch in self._channels.values():
            if ch.reported is not None:
                x = ch.norm_angle(ch.reported[0])
                if ch.reported[1] is not None:
                    v = ch.reported[1]
                else:
                    if ch.prev_meas is None:
                        ch.v_est = 0.0
                    else:
                        v_raw = ch.diff(x, ch.prev_meas) / dt
                        ch.v_est += (1.0 - math.exp(-dt / tau)) * (v_raw - ch.v_est)
                    v = ch.v_est
                if not ch.internal:
                    ch.last_ext = (x, v)
                ch.x_meas, ch.v_meas = x, v
            elif not ch.internal:
                ch.x_meas, ch.v_meas = ch.last_ext
            else:
                ch.x_meas, ch.v_meas = ch.x, ch.v

    def _predict(self, ch: _Channel, x: float, v: float) -> float:
        lat = ch.part.actuator_latency
        return ch.norm_angle(x + v * lat) if lat else x

    # ---- kinematics (spec 3) ------------------------------------------------------------------

    def _frames(self, pose: Mapping[str, float]) -> dict[str, tuple[Frame, Frame]]:
        """Zero-angle frame and rotated frame of each present part, in M."""
        frames: dict[str, tuple[Frame, Frame]] = {}
        f = Frame()
        for part in self.settings.parts():
            f0 = f.then(rpy_matrix(part.offset_rpy), part.offset_xyz)
            if part.name == "eyes":
                frames["eyes"] = (f0, f0)
                break
            yaw = pose[f"{part.name}.yaw"]
            pitch = pose.get(f"{part.name}.pitch", 0.0)
            f = f0.then(joint_matrix(yaw, pitch))
            frames[part.name] = (f0, f)
        return frames

    def _eye_point(self, frames: Mapping[str, tuple[Frame, Frame]]) -> Vec3:
        if "eyes" in frames:
            return frames["eyes"][0].p
        last = self.settings.parts()[-1].name
        return frames[last][1].p

    def _eye_centres(self, eye0: Frame, part: Part) -> dict[str, Vec3]:
        if not part.binocular:
            return {"": eye0.p}
        half = part.ipd / 2.0
        return {
            "left": add(eye0.p, mat_vec(eye0.r, (0.0, half, 0.0))),
            "right": add(eye0.p, mat_vec(eye0.r, (0.0, -half, 0.0))),
        }

    # ---- targets (spec 8) ---------------------------------------------------------------------

    def _split(self, part: Part, axis_name: str, axis: Axis, g: float) -> float:
        e = wrap(g - axis.rest) if axis_name == "yaw" else g - axis.rest
        h: Hinge = axis.split.pos if e >= 0 else axis.split.neg
        reach = self._reach[(part.name, axis_name)][0 if e >= 0 else 1]
        x = abs(e)
        if h.soft == 0:
            share = h.share * max(0.0, x - h.knee)
        else:
            share = h.share * h.soft * (softplus((x - h.knee) / h.soft)
                                        - softplus(-h.knee / h.soft))
        u = max(share, x - reach)
        return axis.rest + sign(e) * u

    def _limit(self, ch: _Channel, raw: float) -> float:
        lim = ch.axis.limits
        if lim is None:
            return wrap(raw)
        lo, hi = lim
        if ch.axis_name == "yaw" and hi - lo > TAU - 1e-9:
            k0 = math.floor((lo - raw) / TAU)
            k1 = math.ceil((hi - raw) / TAU)
            cands = [raw + TAU * k for k in range(k0, k1 + 1) if lo <= raw + TAU * k <= hi]
            if cands:
                return min(cands, key=lambda c: abs(c - ch.x))
        return clamp(raw, lo, hi)

    def _raw_targets(self, part: Part, pose: Mapping[str, float], tgt: Vec3 | None,
                     dt: float) -> tuple[dict[str, float], dict[str, float]]:
        """Raw (limited) targets and their pre-clamp values for one part."""
        keys = [k for k, c in self._channels.items() if c.part is part]
        pre: dict[str, float] = {}
        frames = self._frames(pose)
        f0 = frames[part.name][0]

        if part.name == "eyes":
            centres = self._eye_centres(f0, part)
            for key in keys:
                ch = self._channels[key]
                if tgt is None:
                    pre[key] = ch.axis.rest
                    continue
                side = key.split(".")[1] if part.binocular else ""
                yaw, pitch = direction_angles(f0.to_local(sub(tgt, centres[side])))
                pre[key] = yaw if ch.axis_name == "yaw" else pitch
        elif part.name == "base":
            pre["base.yaw"] = self._base_decision(part, pose, f0, frames, tgt, dt)
        else:
            gaze: dict[str, float] = {}
            if tgt is not None:
                d = f0.to_local(sub(tgt, self._eye_point(frames)))
                gaze["yaw"], gaze["pitch"] = direction_angles(d)
            for key in keys:
                ch = self._channels[key]
                if tgt is None:
                    pre[key] = ch.axis.rest
                else:
                    pre[key] = self._split(part, ch.axis_name, ch.axis, gaze[ch.axis_name])

        raw = {k: self._limit(self._channels[k], v) for k, v in pre.items()}
        return raw, pre

    def _base_decision(self, part: Part, pose: Mapping[str, float], f0: Frame,
                       frames: Mapping[str, tuple[Frame, Frame]], tgt: Vec3 | None,
                       dt: float) -> float:
        if part.external:
            return pose["base.yaw"]
        dec = part.decision
        if tgt is None:
            if self._base_state == "turn":
                log.debug("base: turn -> hold (no target)")
            self._base_state = "hold"
            self._dwell = 0.0
            return self._held
        a, _ = direction_angles(f0.to_local(sub(tgt, self._eye_point(frames))))
        if self._base_state == "hold":
            off = abs(wrap(a - self._held))
            self._dwell = self._dwell + dt if off > dec.dwell_angle else 0.0
            if off > dec.turn_now or self._dwell >= dec.dwell_time:
                self._base_state = "turn"
                why = (f"beyond turn_now {_deg(dec.turn_now):.0f} deg" if off > dec.turn_now
                       else f"held past {_deg(dec.dwell_angle):.0f} deg for {dec.dwell_time:g} s")
                log.debug("base: hold -> turn, target %.1f deg off heading (%s)", _deg(off), why)
        if self._base_state == "turn":
            if abs(wrap(a - pose["base.yaw"])) <= dec.settle_angle:
                self._base_state = "hold"
                self._held = a
                self._dwell = 0.0
                log.debug("base: turn -> hold, facing %.1f deg", _deg(a))
            return a
        return self._held

    # ---- motion (spec 9) ----------------------------------------------------------------------

    def _step(self, ch: _Channel, raw: float, dt: float) -> None:
        dyn = ch.axis.dynamics
        latency = dyn.latency if dyn else 0.0
        deadband = dyn.deadband if dyn else 0.0

        # 9.2 latency gate
        if not ch.engaged:
            if abs(ch.diff(raw, ch.target)) <= deadband:
                ch.target = raw
                ch.timer = 0.0
            else:
                ch.timer += dt
                if ch.timer >= latency:
                    ch.engaged = True
                    ch.target = raw
        else:
            ch.target = raw
            if abs(ch.diff(raw, ch.x_meas)) <= deadband:
                ch.engaged = False
                ch.timer = 0.0

        goal = ch.target

        if dyn is None:  # 9.5 ideal axis
            x_new = self._limit(ch, goal)
            ch.v = ch.diff(x_new, ch.x) / dt
            ch.a = 0.0
            ch.x = x_new
            ch.lead = ch.diff(ch.x, ch.x_meas)
            return

        # 9.4 integrator
        n = max(1, math.ceil(dt / self.settings.max_substep - SUBSTEP_EPS))
        h = dt / n
        w = dyn.omega
        x, v, a = ch.x, ch.v, ch.a
        for _ in range(n):
            err = ch.diff(goal, x)
            a = w * w * err - 2.0 * w * v
            if dyn.a_max is not None:
                a = clamp(a, -dyn.a_max, dyn.a_max)
            v = clamp(v + a * h, -dyn.v_max, dyn.v_max)
            x = ch.norm_angle(x + v * h)

        # 9.3 governor
        ch.lead = ch.diff(x, ch.x_meas)
        governed = dyn.max_lead is not None and abs(ch.lead) > dyn.max_lead
        if governed != ch.governed:
            if governed:
                log.debug("%s: plan is %.1f deg ahead of the measured joint; holding it to "
                          "max_lead %.1f deg", ch.key, _deg(ch.lead), _deg(dyn.max_lead or 0.0))
            else:
                log.debug("%s: measured joint caught up; governor released", ch.key)
            ch.governed = governed
        if governed and dyn.max_lead is not None:
            x = ch.norm_angle(ch.x_meas + sign(ch.lead) * dyn.max_lead)
            v = ch.v_meas

        # 9.6 limits
        if ch.axis.limits is not None:
            lo, hi = ch.axis.limits
            if x < lo:
                x, v, a = lo, max(v, 0.0), max(a, 0.0)
            elif x > hi:
                x, v, a = hi, min(v, 0.0), min(a, 0.0)
        ch.x, ch.v, ch.a = x, v, a

    # ---- output (spec 10) -----------------------------------------------------------------------

    def _output(self, pose: Mapping[str, float], tgt: Vec3 | None, reachable: bool) -> Output:
        setpoints = {
            k: Setpoint(c.x, c.v, c.a) for k, c in self._channels.items() if c.internal
        }
        frames = self._frames(pose)
        last = self.settings.parts()[-1]
        vergence = None
        if last.name == "eyes":
            eye0 = frames["eyes"][0]
            dirs: list[Vec3] = []
            sides = ("left", "right") if last.binocular else ("",)
            for side in sides:
                prefix = f"eyes.{side}." if side else "eyes."
                f = eye0.then(joint_matrix(pose[prefix + "yaw"], pose[prefix + "pitch"]))
                dirs.append(f.facing())
            gaze = dirs[0] if len(dirs) == 1 else add(dirs[0], dirs[1])
            if len(dirs) == 2:
                vergence = angle_between(dirs[0], dirs[1])
        else:
            gaze = frames[last.name][1].facing()
        err = 0.0
        if tgt is not None:
            err = angle_between(gaze, sub(tgt, self._eye_point(frames)))
        base_state = None
        if self.settings.base is not None and not self.settings.base.external:
            base_state = self._base_state
        diag = Diagnostics(
            gaze_error=err,
            reachable=reachable,
            lead={k: c.lead for k, c in self._channels.items() if c.internal},
            base_state=base_state,
            vergence=vergence,
        )
        return Output(t=self.t, setpoints=setpoints, diagnostics=diag, pose=dict(pose))
