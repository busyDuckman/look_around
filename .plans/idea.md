# A small model of natural head and eye movement

Given a point in 3D space to look at, this model says how the head and eyes should rotate so the look appears natural: the eyes do part of the work and the head only goes part of the way, and the head moves smoothly. There is no learning and no data at run time, just a few published ratios and a spring.


It has two parts:

1. Gaze split: given the direction to the target, how much of the look the head does and how much the eyes do.
2. Motion: how the head gets to its target (critically damped spring, speed limit).


## Conventions

- Angles in degrees, relative to the body (torso) forward direction.
- yaw: + is to one side (left in a right-handed z-up world), wraps at +-180.
- pitch: + is up, 0 is level.
- The gaze direction is the direction from the eyes to the target. The head direction is where the face points. The eye-in-head angle is gaze minus head.
- The rest pose is head yaw 0, head pitch REST_PITCH = -8 (looking forward, slightly down).


## 1. Gaze split: head_share

People don't point their head at what they look at. The eyes take part of every gaze shift, and the split depends on direction. head_share(gaze_yaw, gaze_pitch) returns the head's (yaw, pitch).

Yaw: the head takes 69% of the target's azimuth (Kim, Reed & Martin 2010).

    head_yaw = sign(yaw) * max(0.69 * |yaw|, |yaw| - EYE_MAX)

Pitch is measured from the rest pose: e = gaze_pitch - REST_PITCH.

Up (e > 0): the head takes 52% of the elevation (Kim et al. 2010).

    head_pitch = REST_PITCH + max(0.52 * e, e - EYE_MAX)

Down (e < 0): a soft hinge. The eyes alone cover the first DOWN_KNEE = 10 deg below rest, then the head takes DOWN_SHARE = 1.0 (all) of anything further. The corner is rounded with a softplus of width DOWN_SOFT = 4 deg so there is no kink in the motion.

    depth = -e
    z = (depth - DOWN_KNEE) / DOWN_SOFT
    head_pitch = REST_PITCH - max(DOWN_SHARE * DOWN_SOFT * softplus(z), depth - EYE_MAX)

The max(..., x - EYE_MAX) terms say the eyes can only rotate about EYE_MAX = 35 deg in the head, so on big shifts the head must cover at least the remainder.

Why down is different: Kim et al. measured only about 8% head contribution for downward targets, but that was seated subjects making brief glances. For anyone looking at something low for more than a moment it is far too little and looks wrong (eyes rolled down, head level). The hinge follows the form of Hu et al. (CHI 2026, "The Eye-Head Mover Spectrum"): at rest the eyes already sit about 10 deg below the head (Mehrotra et al. 2025), small vertical shifts use no head at all (Fang et al. 2015), and people avoid holding the eyes more than about 30 deg off centre (Stahl 1999), so beyond the knee the head follows.

What it gives (head angle for a gaze angle; yaw with pitch at rest, pitch with yaw 0):

| gaze yaw | 5 | 10 | 20 | 30 | 45 | 60 | 90 |
|---|---|---|---|---|---|---|---|
| head yaw | 3.4 | 6.9 | 13.8 | 20.7 | 31.0 | 41.4 | 62.1 |

| gaze pitch | 30 | 20 | 10 | 0 | -8 (rest) | -18 | -23 | -30 | -42 |
|---|---|---|---|---|---|---|---|---|---|
| head pitch | 11.8 | 6.6 | 1.4 | -3.8 | -8.3 | -10.8 | -14.0 | -20.2 | -32.0 |

(The softplus never quite reaches zero, so at rest the head sits 0.3 deg lower than REST_PITCH. Harmless.)

In the original the head pitch target is then clamped to [-35, 30] deg, a neck range limit.

### The eyes

The original only animates the head; the eyes were not rendered. For a character or robot with eyes, the eye-in-head angle is what is left over:

    eye_yaw   = gaze_yaw   - head_yaw
    eye_pitch = gaze_pitch - head_pitch

clamped to the eye range (about 35 deg).


## 2. Motion

The head's current (yaw, pitch) is driven towards the target (yaw, pitch) independently per axis.

A critically damped spring (no overshoot) with a velocity limit, integrated with semi-implicit Euler:

    err = target - x            (wrapped to +-180 for yaw)
    v  += (OMEGA^2 * err - 2 * OMEGA * v) * dt
    v   = clamp(v, -MAX_SPEED, MAX_SPEED)
    x  += v * dt

OMEGA = 5 rad/s, MAX_SPEED = 90 deg/s. With these, a step settles to 95% in about 0.95 s. The unclamped peak speed is about 1.84 times the step size per second (OMEGA / e), so steps under about 50 deg are pure spring and larger ones hit the speed limit and move at a constant 90 deg/s in the middle.

Head speed scaling (not built): human head peak velocity grows with shift amplitude (a "main sequence"). Scaling OMEGA or MAX_SPEED with step size would make big looks snappier and small ones lazier.


## Per frame

For a look-at point in world space:

1. Gaze angles: direction from the eyes to the point, as (yaw, pitch) relative to the body. Recompute every frame, so a moving point, or the body moving past it, is followed.
2. head_share gives the head target (yaw, pitch); clamp the head pitch to [-35, 30].
3. Step the spring; the result is the head's current (yaw, pitch).
4. The eyes get gaze minus current head, clamped to EYE_MAX.

With nothing to look at, use the rest pose as the head target directly (no gaze split).



## Reference code

The two reusable pieces, free of any engine. Directions are numpy vectors in a z-up world; body_fwd is a horizontal unit vector.

```python
import math
import numpy as np

REST_PITCH = -8.0
HMCR_YAW, HMCR_UP = 0.69, 0.52
DOWN_KNEE, DOWN_SOFT, DOWN_SHARE = 10.0, 4.0, 1.0
EYE_MAX = 35.0


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def gaze_angles(eye, body_fwd, point):
    """(yaw, pitch) of a world point, deg, relative to the body."""
    d = point - eye
    body = math.degrees(math.atan2(body_fwd[1], body_fwd[0]))
    yaw = wrap(math.degrees(math.atan2(d[1], d[0])) - body)
    pitch = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
    return yaw, pitch


def head_share(yaw, pitch):
    """Head (yaw, pitch) for a gaze at (yaw, pitch), deg relative to the body."""
    hy = max(HMCR_YAW * abs(yaw), abs(yaw) - EYE_MAX)
    e = pitch - REST_PITCH
    if e > 0:
        hp = max(HMCR_UP * e, e - EYE_MAX)
    else:   # soft hinge: DOWN_SOFT * softplus((depth - knee) / DOWN_SOFT)
        z = (-e - DOWN_KNEE) / DOWN_SOFT
        softplus = z + math.log1p(math.exp(-z)) if z > 0 else math.log1p(math.exp(z))
        hp = -max(DOWN_SHARE * DOWN_SOFT * softplus, -e - EYE_MAX)
    return math.copysign(hy, yaw), REST_PITCH + hp


class HeadSpring:
    """Head yaw/pitch (deg, relative to the body) following a target smoothly."""
    OMEGA = 5.0         # rad/s, critically damped
    MAX_SPEED = 90.0    # deg/s

    def __init__(self, fps):
        self.dt = 1.0 / fps
        self.yaw, self.pitch = 0.0, REST_PITCH
        self.vyaw = self.vpitch = 0.0

    def step(self, ty, tp):
        tp = max(-35.0, min(30.0, tp))
        w, dt = self.OMEGA, self.dt
        for a, v, tgt in (('yaw', 'vyaw', ty), ('pitch', 'vpitch', tp)):
            x, vel = getattr(self, a), getattr(self, v)
            err = wrap(tgt - x) if a == 'yaw' else tgt - x
            vel += (w * w * err - 2 * w * vel) * dt
            vel = max(-self.MAX_SPEED, min(self.MAX_SPEED, vel))
            setattr(self, v, vel)
            setattr(self, a, x + vel * dt)
        return self.yaw, self.pitch
```


## How well it is checked

- The yaw and upward shares are published averages (Kim et al.), not fitted here.
- The downward hinge (knee 10, softness 4, share 1.0) and all the motion numbers are hand-picked and judged by eye on rendered crowds. They look natural for walking adults at a distance; they have not been compared against motion capture.
- The only quantitative check is the pitch table above, against the cited eye-head literature.


## References

- Kim, Reed, Martin (2010). A model of head movement contribution for gaze transitions. Ergonomics 53(4). https://pubmed.ncbi.nlm.nih.gov/20309742/
- Hu, Sidenmark, Lee, Gellersen (2026). The Eye-Head Mover Spectrum. CHI 2026. https://arxiv.org/html/2602.06164v1
- Fang et al. (2015). Eye-Head Coordination for Visual Cognitive Processing. PLoS One. https://doi.org/10.1371/journal.pone.0121035
- Mehrotra et al. (2025). Sit, stand, and swivel. PLoS One. https://doi.org/10.1371/journal.pone.0334182
- Stahl (1999). Amplitude of human head movements associated with horizontal saccades. Exp Brain Res 126. (cited via Hu et al. 2026)
