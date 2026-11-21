"""Settings document (spec section 4): loading, defaults and unit conversion.

Settings files are YAML (JSON is also accepted). The JSON Schema in schema/settings.schema.json
is the full structural check; this module checks what it needs to run and converts every angle
field to radians.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from importlib import resources
from pathlib import Path
from typing import Any

import yaml

SPEC_MAJOR = 0
SPEC_MINOR = 2

Json = dict[str, Any]


class SettingsError(ValueError):
    pass


@dataclass(frozen=True)
class Share:
    """How much of the gaze a part takes on one side of rest (spec 8.1)."""

    share: float = 1.0
    dead_zone: float = 0.0
    softness: float = 0.0


@dataclass(frozen=True)
class Split:
    pos: Share = field(default_factory=Share)
    neg: Share = field(default_factory=Share)


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
            raise SettingsError("heading.decision: settle_angle must be smaller than turn_now")


@dataclass(frozen=True)
class Part:
    """A joint in the chain: the heading, a torso segment, the head or the eyes."""

    key: str                      # key prefix: "heading", "torso.0", "head", "eyes"
    kind: str                     # "heading", "torso", "head" or "eyes"
    yaw: Axis
    pitch: Axis | None = None
    name: str = ""                # readable name for logs (torso segments may set one)
    external: bool = False
    offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    offset_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    actuator_latency: float = 0.0
    binocular: bool = False
    ipd: float = 0.063
    decision: Decision = field(default_factory=Decision)

    @property
    def label(self) -> str:
        return f"{self.key} ({self.name})" if self.name and self.name != self.key else self.key

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
    heading: Part | None = None
    torso: tuple[Part, ...] = ()
    head: Part | None = None
    eyes: Part | None = None

    def joints(self) -> list[Part]:
        """Torso segments, head and eyes that are present, from the root outwards."""
        return [*self.torso, *(p for p in (self.head, self.eyes) if p is not None)]

    def parts(self) -> list[Part]:
        """Every present part including the heading, from the root outwards."""
        return ([self.heading] if self.heading is not None else []) + self.joints()

    @classmethod
    def load(cls, source: str | Path | Json) -> Settings:
        if isinstance(source, dict):
            return parse(source)
        path = Path(source)
        text = path.read_text(encoding="utf-8")
        doc = json.loads(text) if path.suffix.lower() == ".json" else yaml.safe_load(text)
        if not isinstance(doc, dict):
            raise SettingsError(f"{path}: expected a mapping at the top level")
        return parse(doc)  # pyright: ignore[reportUnknownArgumentType]


# ---- parsing helpers ---------------------------------------------------------------------------

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


def _share(d: Json | None, k: float, where: str) -> Share:
    if d is None:
        return Share()
    for old in ("knee", "soft"):
        if old in d:
            raise SettingsError(f"{where}: '{old}' was renamed (use dead_zone / softness)")
    return Share(share=_num(d, "share", None), dead_zone=_num(d, "dead_zone", 0.0, k),
                 softness=_num(d, "softness", 0.0, k))


def _axis(d: Json, k: float, where: str) -> Axis:
    lim = d.get("limits")
    limits = None
    if lim is not None:
        limits = (float(lim[0]) * k, float(lim[1]) * k)
        if not limits[0] < limits[1]:
            raise SettingsError(f"{where}: limits must be [min, max] with min < max")
    split = Split()
    if d.get("split") is not None:
        split = Split(pos=_share(d["split"].get("pos"), k, where),
                      neg=_share(d["split"].get("neg"), k, where))
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


def _part(key: str, kind: str, d: Json, k: float) -> Part:
    if "yaw" not in d:
        raise SettingsError(f"{key}: yaw is required")
    has_pitch = kind in ("head", "eyes")
    if has_pitch and "pitch" not in d:
        raise SettingsError(f"{key}: pitch is required")
    if not has_pitch and "pitch" in d:
        raise SettingsError(f"{key}: has no pitch axis (only the head and eyes pitch)")
    yaw = _axis(d["yaw"], k, f"{key}.yaw")
    pitch = _axis(d["pitch"], k, f"{key}.pitch") if has_pitch else None
    if pitch is not None and pitch.limits is None:
        raise SettingsError(f"{key}.pitch: pitch must have limits")
    if kind == "eyes" and yaw.limits is None:
        raise SettingsError("eyes.yaw: eyes must have limits")
    if kind == "heading" and "offset" in d:
        raise SettingsError("heading: has no offset (the root is its frame)")
    offset: Json = d.get("offset") or {}
    dec: Json = d.get("decision") or {}
    return Part(
        key=key,
        kind=kind,
        yaw=yaw,
        pitch=pitch,
        name=str(d.get("name", "")),
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
    if major != SPEC_MAJOR or (major == 0 and minor != SPEC_MINOR):
        supported = f"{SPEC_MAJOR}.{SPEC_MINOR}"
        raise SettingsError(f"spec_version {version} is not supported (reads {supported})")
    if "base" in doc:
        raise SettingsError("'base' was renamed to 'heading'")
    unit = doc.get("angle_unit", "rad")
    if unit not in ("rad", "deg"):
        raise SettingsError("angle_unit must be 'rad' or 'deg'")
    k = math.pi / 180.0 if unit == "deg" else 1.0

    heading = _part("heading", "heading", doc["heading"], k) if doc.get("heading") else None
    torso_docs: Any = doc.get("torso") or []
    if not isinstance(torso_docs, list):
        raise SettingsError("torso must be a list of segments")
    segments: list[Json] = list(torso_docs)  # pyright: ignore[reportUnknownArgumentType]
    torso = tuple(_part(f"torso.{i}", "torso", seg, k) for i, seg in enumerate(segments))
    head = _part("head", "head", doc["head"], k) if doc.get("head") else None
    eyes = _part("eyes", "eyes", doc["eyes"], k) if doc.get("eyes") else None
    if not torso and head is None and eyes is None:
        raise SettingsError("at least one joint (torso segment, head or eyes) must be present")
    return Settings(
        spec_version=version,
        name=str(doc.get("name", "")),
        description=str(doc.get("description", "")),
        max_substep=_num(doc, "max_substep", 1.0 / 120.0),
        velocity_filter_tau=_num(doc, "velocity_filter_tau", 0.02),
        heading=heading,
        torso=torso,
        head=head,
        eyes=eyes,
    )


def preset_path(name: str) -> Path:
    """Locate a shipped preset: inside the wheel, or the repo's presets/ in a checkout."""
    filename = f"{name}.yaml"
    packaged = resources.files("lookaround").joinpath("presets", filename)
    if packaged.is_file():
        return Path(str(packaged))
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "presets" / filename
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"no preset named {name!r}")


def load_preset(name: str) -> Settings:
    return Settings.load(preset_path(name))
