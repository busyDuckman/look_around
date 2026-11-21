from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
PRESETS = sorted(p.stem for p in (REPO / "presets").glob("*.yaml"))


def preset_doc(name: str) -> dict[str, Any]:
    return yaml.safe_load((REPO / "presets" / f"{name}.yaml").read_text(encoding="utf-8"))


def deg(x: float) -> float:
    return math.degrees(x)


@pytest.fixture(params=PRESETS)
def preset_name(request: pytest.FixtureRequest) -> str:
    return str(request.param)
