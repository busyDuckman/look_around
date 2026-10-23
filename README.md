# lookaround

A library for generating natural-looking motion for anything that can look around (people, robots, animals).

The goal is a really simple and intuitive control interface, a .json that defines the system (constraints, speeds, etc.), a powerful backend based on existing research that does the rest. The system is a type of augmented control, allowing a thing to look at a target while performing other tasks.

It's suitable for both animation and control systems (PID loop).

# The Model

A cascading set of transforms work together to direct gaze. The model can be any/all of the following:

  - eyes: move to point directly at the target
  - head: is involved if the eyes can't get there comfortably
  - torso[n]: yaw adjustments, achieved by body movement, if the head can't get there comfortably.
  - heading: yaw adjustment of the character/robot (i.e. "please turn to face")

The model is opinionated in that pitch adjustments are only done via the eyes and head.
Every part of the kinematic chain beneath that is given a yaw (heading) only.

The library takes feedback, so if other things are going on and another event is driving part of the robot/animation-rig then the rest of the system will compensate. Also parts of the system can be set to an 'external' mode, so that the look-at controller will respond naturally to other movement in the animation-rig/robot.

This setup is sufficient for natural-looking movement across a great many systems: people, dogs, snakes, owls, assorted robots. It will also respond to constraints, such as a person being seated.



## The interface


The setup file specifies if these things exist, what their range of movement is, and if it's something the library controls or reacts to.



## The settings file


You describe the looker in a settings file: which parts it has (eyes, head, torso, base) and their limits. Each tick you give it a target point and, optionally, the measured pose of its joints. It returns setpoints for the joints it drives. The eyes take small shifts, the head takes a share, the torso joins on large shifts, and the base turns when the target stays far off to the side. Shares and limits come from published eye-head-torso studies.

lookaround does not do motor control. Its setpoints feed your own PID loops, servos or animation. If a joint doesn't get where it was asked (lag, a joint held by another controller, or a part moved by something else), the parts further out make up the difference.

Status: early development. The spec and the Python reference implementation exist; C++, Rust, C# and TypeScript ports are planned.



## Example

```python

# in this example a robot sits in a chair and looks around


import lookaround as la

looker = la.Looker.from_preset("human_sitting")

while running:

    # Update measured pose (closed loop)
    looker.feedback_roll_pitch_yaw(LookerParts::base, [measured_torso_yaw, 0, 0])   

    # Is the ball an obtainable target
    if looker.reachable(ball_position):
        out = looker.update(time_delta, ball_position)   # look at the ball
    else:
        out = looker.update(time_delta, None)           # return to neutral
    
    # update system
    for joint, sp in out.setpoints.items():          # "head.yaw", "eyes.pitch", ...
        controller[joint].set(sp.position, sp.velocity)
    

```

## Presets

| Preset | Parts |
|---|---|
| `human_standing` | eyes, head, torso, base (feet) |
| `human_seated` | eyes, head, limited torso; no base |
| `human_swivel` | as standing, with a swivel chair as the base |
| `head_only` | eyes and head (the original model in `.plans/idea.md`) |
| `pan_tilt` | head only, continuous pan, acceleration-limited |
| `owl` | head only, about 270° of yaw |

Any part can be left out (`null`) or marked `"external": true`, meaning something else drives it and lookaround only reads it.

## Layout

```
docs/spec.md                  the specification every implementation follows
schema/settings.schema.json   JSON Schema for settings files
presets/                      shared settings presets
python/                       Python reference implementation and tests
testbed/animation/            testbed A: Python server driving the browser viewer
web/testbed/                  browser viewer (three.js mannequin)
.plans/                       plan, research notes, original idea
```

## Running

Python 3.10+. From the repo root:

```
python -m venv python/.venv
python/.venv/Scripts/pip install -e "./python[test,testbed]"   # bin/ instead of Scripts/ on Linux/macOS
python/.venv/Scripts/python -m pytest python
python/.venv/Scripts/python testbed/animation/server.py        # opens the testbed in a browser
```

In the testbed, pick a preset and a ball path, or drag the ball. Add `--debug` to print lookaround's diagnostic log.

## Licence

MIT OR Apache-2.0 (licence files to be added).
