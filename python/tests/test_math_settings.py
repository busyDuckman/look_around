from __future__ import annotations

import copy
import json
import math
from typing import Any

import pytest
from conftest import REPO, preset_doc
from jsonschema import Draft202012Validator

import lookaround as la
from lookaround._math import softplus, wrap


def test_wrap_interval() -> None:
    assert wrap(math.pi) == pytest.approx(math.pi)
    assert wrap(-math.pi) == pytest.approx(math.pi)
    assert wrap(1.5 * math.pi) == pytest.approx(-0.5 * math.pi)
    assert wrap(-7.0) == pytest.approx(-7.0 + 2 * math.pi)
    for a in (-10.0, -3.2, 0.0, 3.1, 9.9):
        assert -math.pi < wrap(a) <= math.pi


def test_softplus_stable() -> None:
    assert softplus(0.0) == pytest.approx(math.log(2))
    assert softplus(800.0) == pytest.approx(800.0)
    assert softplus(-800.0) == pytest.approx(0.0)


def test_presets_match_schema(preset_name: str) -> None:
    schema = json.loads((REPO / "schema" / "settings.schema.json").read_text(encoding="utf-8"))
    errors = list(Draft202012Validator(schema).iter_errors(preset_doc(preset_name)))
    assert not errors, [e.message for e in errors]


def test_presets_load(preset_name: str) -> None:
    s = la.load_preset(preset_name)
    assert s.name == preset_name
    assert s.joints()


def test_degrees_converted_and_named() -> None:
    s = la.load_preset("human_standing")
    assert s.head is not None and s.head.pitch is not None
    assert s.head.pitch.rest == pytest.approx(math.radians(-8))
    assert s.head.yaw.split.pos.dead_zone == pytest.approx(math.radians(12))
    assert s.head.yaw.dynamics is not None
    assert s.head.yaw.dynamics.v_max == pytest.approx(math.radians(90))
    assert s.head.yaw.dynamics.omega == pytest.approx(5.0)  # not an angle
    assert s.heading is not None
    assert s.heading.decision.turn_now == pytest.approx(math.radians(100))
    assert [p.name for p in s.torso] == ["pelvis", "chest"]
    assert [p.key for p in s.torso] == ["torso.0", "torso.1"]


def test_json_settings_still_load(tmp_path: Any) -> None:
    path = tmp_path / "pan_tilt.json"
    path.write_text(json.dumps(preset_doc("pan_tilt")), encoding="utf-8")
    assert la.Settings.load(path).name == "pan_tilt"


def _no_joints(d: dict[str, Any]) -> None:
    d.update(torso=[], head=None, eyes=None)


@pytest.mark.parametrize(
    "mutate",
    [
        lambda d: d.update(spec_version="1.0.0"),
        lambda d: d.update(spec_version="0.1.0"),
        lambda d: d["torso"][0].update(pitch={"limits": [-1, 1]}),
        lambda d: d["head"]["pitch"].update(limits=None),
        lambda d: d["head"]["yaw"].update(limits=[10, -10]),
        lambda d: d["head"]["yaw"]["split"].update(pos={"share": 0.69, "knee": 12}),
        lambda d: d.update(base=d.pop("heading")),
        lambda d: d["heading"].update(offset={"xyz": [0, 0, 1]}),
        _no_joints,
    ],
)
def test_bad_settings_rejected(mutate) -> None:  # type: ignore[no-untyped-def]
    d = copy.deepcopy(preset_doc("human_standing"))
    mutate(d)
    with pytest.raises(la.SettingsError):
        la.Settings.load(d)
