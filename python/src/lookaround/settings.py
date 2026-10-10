"""Settings document (spec section 4): parsing, defaults and unit conversion.

The JSON Schema in schema/settings.schema.json is the full structural check; this module
checks what it needs to run and converts every angle field to radians.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from importlib import resources
from pathlib import Path
from typing import Any

SPEC_MAJOR = 0
SPEC_MINOR = 1
PART_NAMES = ("base", "torso", "head", "eyes")

Json = dict[str, Any]


class SettingsError(ValueError):
    pass


@dataclass(frozen=True)
class Hinge:
    share: float = 1.0
    knee: float = 0.0
    soft: float = 0.0


@dataclass(frozen=True)
class Split:
    pos: Hinge = field(default_factory=Hinge)
    neg: Hinge = field(default_factory=Hinge)


@dataclass(frozen=True)
class Dynamics:
    omega: float
    v_max: float
    a_max: float | None = None
    latency: float = 0.0
    deadband: float = 0.0
    max_lead: float | None = None


@dataclass(frozen=True)
class Axis:
    limits: tuple[float, float] | None = None
    rest: float = 0.0
    split: Split = field(default_factory=Split)
    dynamics: Dynamics | None = None

    @property
    def continuous(self) -> bool:
        return self.limits is None


@dataclass(frozen=True)
class Decision:
    turn_now: float = math.radians(100.0)
    dwell_angle: float = math.radians(30.0)
    dwell_time: float = 1.5
    settle_angle: float = math.radians(5.0)

    def __post_init__(self) -> None:
        if self.settle_angle >= self.turn_now:
            raise SettingsError("base.decision: settle_angle must be smaller than turn_now")


@dataclass(frozen=True)
class Part:
    name: str
    yaw: Axis
    pitch: Axis | None = None
    external: bool = False
    offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    offset_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    actuator_latency: float = 0.0
    binocular: bool = False
    ipd: float = 0.063
    decision: Decision = field(default_factory=Decision)

    def axes(self) -> dict[str, Axis]:
        out = {"yaw": self.yaw}
        if self.pitch is not None:
            out["pitch"] = self.pitch
        return out


@dataclass(frozen=True)
class Settings:
    spec_version: str
    name: str
    description: str = ""
    max_substep: float = 1.0 / 120.0
    velocity_filter_tau: float = 0.02
    base: Part | None = None
    torso: Part | None = None
    head: Part | None = None
    eyes: Part | None = None

    def parts(self) -> list[Part]:
        """Non-null parts, base first."""
        return [p for p in (self.base, self.torso, self.head, self.eyes) if p is not None]

    @classmethod
    def load(cls, source: str | Path | Json) -> Settings:
        if isinstance(source, dict):
            return parse(source)
        return parse(json.loads(Path(source).read_text(encoding="utf-8")))


def _num(d: Json, key: str, default: float | None, k: float = 1.0) -> float:
    v = d.get(key, default)
    if v is None:
        raise SettingsError(f"missing required field {key!r}")
    return float(v) * k


def _opt(d: Json, key: str, k: float) -> float | None:
    v = d.get(key)
    return None if v is None else float(v) * k


def _angle_or(d: Json, key: str, default_deg: float, k: float) -> float:
    """An angle field whose spec default is given in degrees whatever the file's unit."""
    return float(d[key]) * k if key in d else math.radians(default_deg)


def _vec3(v: Any, k: float = 1.0) -> tuple[float, float, float]:
    if v is None:
        return (0.0, 0.0, 0.0)
    if len(v) != 3:
        raise SettingsError("expected a 3-vector")
    return (float(v[0]) * k, float(v[1]) * k, float(v[2]) * k)


def _hinge(d: Json | None, k: float) -> Hinge:
    if d is None:
        return Hinge()
    return Hinge(share=_num(d, "share", None), knee=_num(d, "knee", 0.0, k),
                 soft=_num(d, "soft", 0.0, k))


def _axis(d: Json, k: float, where: str) -> Axis:
    lim = d.get("limits")
    limits = None
    if lim is not None:
        limits = (float(lim[0]) * k, float(lim[1]) * k)
        if not limits[0] < limits[1]:
            raise SettingsError(f"{where}: limits must be [min, max] with min < max")
    split = Split()
    if d.get("split") is not None:
        split = Split(pos=_hinge(d["split"].get("pos"), k), neg=_hinge(d["split"].get("neg"), k))
    dyn = None
    if d.get("dynamics") is not None:
        dd = d["dynamics"]
        dyn = Dynamics(
            omega=_num(dd, "omega", None),
            v_max=_num(dd, "v_max", None, k),
            a_max=_opt(dd, "a_max", k),
            latency=_num(dd, "latency", 0.0),
            deadband=_num(dd, "deadband", 0.0, k),
            max_lead=_opt(dd, "max_lead", k),
        )
    rest = _num(d, "rest", 0.0, k)
    if limits is not None and not limits[0] <= rest <= limits[1]:
        raise SettingsError(f"{where}: rest outside limits")
    return Axis(limits=limits, rest=rest, split=split, dynamics=dyn)


def _part(name: str, d: Json, k: float) -> Part:
    if "yaw" not in d:
        raise SettingsError(f"{name}: yaw is required")
    has_pitch = name in ("head", "eyes")
    if has_pitch and "pitch" not in d:
        raise SettingsError(f"{name}: pitch is required")
    if not has_pitch and "pitch" in d:
        raise SettingsError(f"{name}: has no pitch axis (yaw only)")
    yaw = _axis(d["yaw"], k, f"{name}.yaw")
    pitch = _axis(d["pitch"], k, f"{name}.pitch") if has_pitch else None
    if pitch is not None and pitch.limits is None:
        raise SettingsError(f"{name}.pitch: pitch must have limits")
    if name == "eyes" and yaw.limits is None:
        raise SettingsError("eyes.yaw: eyes must have limits")
    offset: Json = d.get("offset") or {}
    dec: Json = d.get("decision") or {}
    return Part(
        name=name,
        yaw=yaw,
        pitch=pitch,
        external=bool(d.get("external", False)),
        offset_xyz=_vec3(offset.get("xyz")),
        offset_rpy=_vec3(offset.get("rpy"), k),
        actuator_latency=_num(d, "actuator_latency", 0.0),
        binocular=bool(d.get("binocular", False)),
        ipd=_num(d, "ipd", 0.063),
        decision=Decision(
            turn_now=_angle_or(dec, "turn_now", 100.0, k),
            dwell_angle=_angle_or(dec, "dwell_angle", 30.0, k),
            dwell_time=_num(dec, "dwell_time", 1.5),
            settle_angle=_angle_or(dec, "settle_angle", 5.0, k),
        ),
    )


def parse(doc: Json) -> Settings:
    version = str(doc.get("spec_version", ""))
    try:
        major, minor = (int(x) for x in version.split("-")[0].split(".")[:2])
    except ValueError as e:
        raise SettingsError(f"bad spec_version {version!r}") from e
    if major != SPEC_MAJOR:
        raise SettingsError(f"spec_version {version} is not supported (major {SPEC_MAJOR})")
    if minor > SPEC_MINOR:
        import warnings

        warnings.warn(f"settings target spec {version}, newer than this library", stacklevel=2)
    unit = doc.get("angle_unit", "rad")
    if unit not in ("rad", "deg"):
        raise SettingsError("angle_unit must be 'rad' or 'deg'")
    k = math.pi / 180.0 if unit == "deg" else 1.0
    parts = {n: (_part(n, doc[n], k) if doc.get(n) is not None else None) for n in PART_NAMES}
    if all(p is None for p in parts.values()):
        raise SettingsError("at least one part must be non-null")
    return Settings(
        spec_version=version,
        name=str(doc.get("name", "")),
        description=str(doc.get("description", "")),
        max_substep=_num(doc, "max_substep", 1.0 / 120.0),
        velocity_filter_tau=_num(doc, "velocity_filter_tau", 0.02),
        **parts,
    )


def preset_path(name: str) -> Path:
    """Locate a shipped preset: inside the wheel, or the repo's presets/ in a checkout."""
    packaged = resources.files("lookaround").joinpath("presets", f"{name}.json")
    if packaged.is_file():
        return Path(str(packaged))
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "presets" / f"{name}.json"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"no preset named {name!r}")


def load_preset(name: str) -> Settings:
    return Settings.load(preset_path(name))
