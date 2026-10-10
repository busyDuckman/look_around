# lookaround (Python reference implementation)

Natural gaze for anything that can look around: eyes, head, torso and base, described by one settings file. This package is the reference implementation of [docs/SPEC.md](../docs/SPEC.md).

```python
import lookaround as la

looker = la.Looker.from_preset("human_standing")

while running:
    looker.report({"torso.yaw": measured_torso})   # optional, for closed loop
    out = looker.update(dt, ball_position)          # point in the mount frame, or None
    for joint, sp in out.setpoints.items():          # e.g. "head.yaw", "eyes.pitch"
        controller[joint].set(sp.position, sp.velocity)
```

No dependencies. Python 3.10+.

## Development

```
python -m venv .venv
.venv/Scripts/pip install -e ".[test,testbed]"     # Windows; .venv/bin/pip elsewhere
.venv/Scripts/python -m pytest
```
