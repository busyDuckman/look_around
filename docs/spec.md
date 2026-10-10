# lookaround specification

Version: 0.1.0-draft (2026-10-09)

This document defines the behaviour every lookaround implementation must have. The Python package is the reference implementation; the C++, Rust, C# and TypeScript ports must match it within the tolerances defined for each port (section 11). Design background is in `.plans/plan.md`; sources for the numbers are in `.plans/research_notes.md`.

The key words MUST, MUST NOT, SHOULD and MAY are used as in RFC 2119.

## 1. Scope

lookaround computes where the movable parts of a "looker" (eyes, head, torso, base) should point so that it looks at a target in a natural way. Each tick, the application:

1. reports the measured pose of the parts (optional for parts lookaround drives itself),
2. calls `update` with the time step and the target, and
3. receives setpoints for the parts lookaround drives.

lookaround does not do motor control. Setpoints go to the application's own PID loops, servos or animation system.

Out of scope for 0.1: saccade and smooth-pursuit eye dynamics (reserved for 0.2), torso and base pitch or roll, head roll, translation of the base (walking), multiple simultaneous targets.

## 2. Units and numbers

- Lengths in metres, time in seconds, angles in radians in the API.
- A settings file MAY declare `"angle_unit": "deg"`. Implementations MUST then multiply every field marked *angle* in section 4 by π/180 when loading. Fields that are not angles (`omega`, `freq`, `share`, lengths, times) are never converted.
- All arithmetic is IEEE-754 double precision in the reference implementation. Ports MAY offer single precision.
- `wrap(a)` maps an angle to the interval (−π, π]: `wrap(a) = a − 2π·ceil((a − π) / (2π))`.
- `clamp(x, lo, hi) = min(max(x, lo), hi)`.
- `softplus(z) = z + log1p(exp(−z))` if z > 0, else `log1p(exp(z))`. This form avoids overflow.
- `sign(x)` is +1 for x ≥ 0, −1 otherwise.

## 3. Frames and kinematic chain

### 3.1 Mount frame M

The frame the whole looker stands on. Its origin is at the base pivot, z points up, and its orientation does not rotate with the base. For a robot on a mobile base this is typically the odometry frame translated to the robot; for a character it is the world frame translated to the character. If the base part is `null`, M is simply the body's frame.

The target is given in M (section 7).

### 3.2 Axes

Right-handed, x forward, y left, z up (ROS REP 103).

- yaw: rotation about the local z axis. Positive yaw turns the part to its left (counter-clockwise seen from above).
- pitch: positive pitch tilts the facing direction up. A joint pitch of θ is a rotation of −θ about the local y axis.
- A part's facing direction is its local +x axis.

### 3.3 Chain

```
M --offset_base--> base(yaw) --offset_torso--> torso(yaw) --offset_head--> head(yaw, pitch) --offset_eyes--> eyes(yaw, pitch)
```

Each part's angles are relative to its parent. A part that is `null` is removed from the chain: its offset and rotation are the identity, and the next part attaches to its parent. For example, with `torso: null` the head offset is applied directly after the base rotation.

### 3.4 Transforms

The pose of part k's joint frame in its parent frame is

```
T_k = Offset_k · Rz(yaw_k) · Ry(−pitch_k)
```

where `Offset_k = Trans(xyz) · Rz(rpy[2]) · Ry(rpy[1]) · Rx(rpy[0])` (the URDF joint-origin convention). Parts without a pitch axis use pitch_k = 0.

The eyes are treated differently: `offset_eyes` places the eye point (cyclopean point, between the eyes) in the head frame, and eye yaw and pitch rotate the gaze about that point. With `binocular: true`, the left and right eye centres are at the eye point ± (0, ipd/2, 0) in the eye-point frame; left is +y.

### 3.5 Direction to angles

For a vector d expressed in a frame F:

```
yaw(d)   = atan2(d.y, d.x)
pitch(d) = atan2(d.z, hypot(d.x, d.y))
```

Because every joint is a yaw followed by a pitch about the same origin, a direction (yaw, pitch) in a part's parent frame is reached exactly by joint angles (yaw, pitch), apart from the offset translation between the joint origin and the eye point. That translation is accounted for by always measuring directions from the eye point (section 8.2).

## 4. Settings document

A settings document is a JSON object. The JSON Schema in `schema/settings.schema.json` is normative for structure; this section is normative for meaning. Fields marked *angle* are converted when `angle_unit` is `"deg"`.

### 4.1 Top level

| Field | Type | Default | Meaning |
|---|---|---|---|
| `spec_version` | string | required | Spec version the document targets, e.g. `"0.1.0"`. |
| `name` | string | required | Identifier. |
| `description` | string | `""` | Free text. |
| `angle_unit` | `"rad"` or `"deg"` | `"rad"` | Unit for all *angle* fields. |
| `max_substep` | number > 0 (s) | `1/120` | Largest integration step (section 9.4). |
| `velocity_filter_tau` | number > 0 (s) | `0.02` | Time constant of the velocity estimator (section 6.3). |
| `base`, `torso`, `head`, `eyes` | Part or `null` | `null` | The four parts. At least one MUST be non-null. |

### 4.2 Part (common fields)

| Field | Type | Default | Meaning |
|---|---|---|---|
| `external` | boolean | `false` | `true` if something else drives the part (section 5). |
| `offset.xyz` | [x, y, z] (m) | [0, 0, 0] | Translation from the parent joint frame. |
| `offset.rpy` | [r, p, y] (*angle*) | [0, 0, 0] | Fixed rotation from the parent joint frame. |
| `actuator_latency` | number ≥ 0 (s) | 0 | Known delay between a setpoint and the measured response. Used for latency compensation (section 6.4). |
| `yaw` | Axis | required | |
| `pitch` | Axis | required for `head` and `eyes`; MUST be absent for `base` and `torso` | |

Part-specific fields:

- `eyes.binocular` (boolean, default false) and `eyes.ipd` (m, default 0.063).
- `base.decision` (section 8.3): `turn_now` (*angle*, default 100°), `dwell_angle` (*angle*, default 30°), `dwell_time` (s, default 1.5), `settle_angle` (*angle*, default 5°).

### 4.3 Axis

| Field | Type | Default | Meaning |
|---|---|---|---|
| `limits` | [min, max] (*angle*) or `null` | `null` | Hard joint limits. `null` means continuous rotation; only allowed for yaw. |
| `rest` | *angle* | 0 | Rest angle. Idle target; origin for the split. |
| `split` | Split | see below | How much of the gaze this part takes (section 8.1). Only for `torso` and `head`. Default: share 1, knee 0, soft 0 on both sides. |
| `dynamics` | Dynamics or `null` | `null` | `null` means ideal: the setpoint equals the target (section 9.5). Ignored for external parts. |

Split: `{ "pos": Hinge, "neg": Hinge }`. `pos` applies when the gaze is on the positive side of rest (left, or up), `neg` otherwise.

Hinge: `share` (≥ 0), `knee` (*angle*, ≥ 0), `soft` (*angle*, ≥ 0).

Dynamics:

| Field | Type | Default | Meaning |
|---|---|---|---|
| `omega` | number > 0 (1/s) | required | Natural frequency of the critically damped spring. |
| `v_max` | *angle* per second, > 0 | required | Velocity limit. |
| `a_max` | *angle* per second², > 0, or `null` | `null` | Acceleration limit; `null` means none. |
| `latency` | number ≥ 0 (s) | 0 | Onset latency (section 9.2). |
| `deadband` | *angle* ≥ 0 | 0 | Error below which the latency gate re-arms (section 9.2). |
| `max_lead` | *angle* > 0 or `null` | `null` | Reference governor limit (section 9.3); `null` disables it. |

## 5. Part states

| Setting | lookaround reads | lookaround outputs |
|---|---|---|
| `null` | nothing | nothing |
| part object, `external: false` (the default) | measured angles and velocities, if reported | a setpoint per axis |
| part object, `external: true` | measured angles and velocities | nothing |

In this document a non-null part with `external: false` is called internal, and one with `external: true` is called external. There are no requests to the application; an external part is an input only.

## 6. Measurements

### 6.1 Report

Before each `update`, the application MAY call `report(measured)`. A measurement gives, per joint axis, a `position` and optionally a `velocity`. A report applies to the next `update` only.

Joint axes are named by flat keys `<part>.<axis>`: `base.yaw`, `torso.yaw`, `head.yaw`, `head.pitch`, `eyes.yaw`, `eyes.pitch`. With `binocular: true` the eye keys are `eyes.left.yaw`, `eyes.left.pitch`, `eyes.right.yaw` and `eyes.right.pitch`. The same keys are used for setpoints and diagnostics.

### 6.2 Missing values

For each axis at each `update`:

- Internal, not reported (open loop): for the axis's own latency gate and governor, the measured position and velocity are its reference state before this update's step. For the parts further out, the pose used is its reference state after the step (section 8). In open loop this keeps the eyes exactly on target while the head moves, as in idea.md.
- External, not reported: the last reported value is used. Before the first report, rest and zero velocity are used.

### 6.3 Velocity estimate

If a position is reported without a velocity, the velocity is estimated:

```
v_raw = diff(x_now, x_prev) / dt              (diff = wrap(a − b) for continuous axes, a − b otherwise)
alpha = 1 − exp(−dt / velocity_filter_tau)
v_est = v_est + alpha · (v_raw − v_est)
```

x_prev is the measured position used in the previous update; v_est starts at 0. On the first update there is no x_prev and v_est = 0.

### 6.4 Latency compensation

Every computation in section 8 uses the predicted pose:

```
x_pred = x_meas + v_meas · actuator_latency
```

(wrapped for continuous axes). With `actuator_latency = 0` this is the measured pose.

## 7. Target

`update(dt, target)` takes `dt > 0` and a target that is either a point in M (metres) or `null`. `null` means idle (section 8.5).

If the target is closer than 1e-6 m to the eye point, it is treated as `null` for that update.

Implementations MAY provide a helper that accepts the target in the base frame and converts it using the measured base yaw.

## 8. Targets for each part

Parts are processed one at a time from the base outwards: compute the part's raw targets, then step its internal axes (section 9), then move on to the next part. The pose used for the computations is a table of joint angles that starts as the predicted measured pose (6.4) of every axis. After a part has been stepped, each of its internal axes that was not reported is overwritten in the table with its new reference position, predicted by 6.4 with the new reference velocity. Reported and external axes keep their measured value.

So every part works from the measured pose of the parts above it (or, in open loop, from their just-updated reference), never from their setpoints. This is what makes the further-out parts make up for parts that lag, are held, or are external.

### 8.1 Split function

For an axis with rest r, a gaze angle g (section 8.2) and child reach R (section 8.2):

```
e    = wrap(g − r)  for yaw,   g − r  for pitch
side = pos if e ≥ 0 else neg
h(x) = share · max(0, x − knee)                                           if soft = 0
h(x) = share · soft · ( softplus((x − knee)/soft) − softplus(−knee/soft) ) if soft > 0
u    = max( h(|e|), |e| − R_side )
raw  = r + sign(e) · u
```

The subtraction of `softplus(−knee/soft)` makes h(0) = 0, so the split passes exactly through rest. (idea.md leaves this term out, which puts the rest pose about 0.3° low; that offset is removed here on purpose.)

### 8.2 Gaze angle and child reach

The gaze angle g for part k is the (yaw, pitch) (3.5) of the vector from the eye point to the target, expressed in part k's zero-angle joint frame (its parent's frame composed with `Offset_k`). Both the frame and the eye point are computed from the current pose table (section 8 intro).

The eye point is the origin of the eyes' zero-angle frame (head frame composed with `offset_eyes`). If the eyes are null, it is the origin of the furthest-out part's joint frame.

The child reach R_pos (R_neg) of an axis is the sum, over all non-null parts further out that have the same axis, of (max − rest) (respectively (rest − min)) of their limits. If any of them is continuous, R is +∞ and the second term of `u` has no effect.

### 8.3 Base (yaw only)

If the base is internal, its raw target comes from a two-state decision rather than a split. Let a = the gaze yaw (8.2) in the base's zero-angle frame (M itself unless the base has an offset) and H = the held heading (initially the first measured base yaw, or rest).

- HOLD: raw target = H. A dwell timer accumulates dt while |wrap(a − H)| > dwell_angle and resets to 0 otherwise. Switch to TURN if |wrap(a − H)| > turn_now or the timer reaches dwell_time.
- TURN: raw target = a. Switch to HOLD, with H := a and the timer reset, when |wrap(a − x_pred_base)| ≤ settle_angle.
- Idle (target null): stay in or switch to HOLD; the timer resets.

### 8.4 Torso, head, eyes

- Torso yaw: split (8.1) of the gaze yaw in the base frame.
- Head yaw and pitch: split of the gaze yaw and pitch in the torso frame (or the base frame, or M, if the parts above are null).
- Eyes yaw and pitch: the gaze direction in the head frame, from the eye point (or from each eye centre when binocular), clamped to the eye limits. The eyes get the remainder, so they have no split.

### 8.5 Idle

With a null target, every axis's raw target is its rest, except the base, which holds H.

### 8.6 Limits and the long way round

After the split, the raw target is clamped to the axis limits. For a limited yaw axis whose limits span more than 2π − 1e-9, the raw target is first replaced by the candidate raw + 2πk (integer k) that lies within the limits and is closest to the current reference position. For limited axes, all differences in section 9 are plain differences (no wrap), so motion stays inside the limits and goes the long way round when it has to.

## 9. Motion of internal axes

Each internal axis keeps a reference state: position x, velocity v, acceleration a, an active target T and a latency gate. Setpoints are the reference state.

### 9.1 Order of operations for one update

1. Resolve measurements (6.2), estimate velocities (6.3), predict (6.4), and build the pose table.
2. If the target is closer than 1e-6 m to the eye point, treat it as null.
3. For each part from the base outwards: raw targets (section 8); then for each of its internal axes: latency gate (9.2), integrate (9.4), governor (9.3), limits (9.6); then update the pose table.
4. Diagnostics (section 10), from the final pose table.
5. Advance time: t := t + dt.

t starts at 0 and is reported in the output.

### 9.2 Latency gate

Small continuous changes in the target are followed immediately; a large new target only starts the move after `latency` seconds.

```
if not engaged:
    if |diff(raw, T)| ≤ deadband:  T := raw;  timer := 0
    else:
        timer += dt
        if timer ≥ latency:  engaged := true;  T := raw
else:
    T := raw
    if |diff(raw, x_meas)| ≤ deadband:  engaged := false;  timer := 0
```

With `latency = 0` this always gives T = raw.

### 9.3 Reference governor

After integration, if `max_lead` is not null:

```
d = diff(x, x_meas)
if |d| > max_lead:
    x := x_meas + sign(d) · max_lead
    v := v_meas
```

In open loop x_meas = x, so the governor never acts.

### 9.4 Integrator

The step is split into n = max(1, ceil(dt / max_substep − 1e-9)) equal substeps h = dt / n (the 1e-9 keeps rounding error from adding a substep when dt is a multiple of max_substep). Measured values are held constant across substeps. Each substep:

```
err = diff(T, x)
a   = omega² · err − 2 · omega · v
a   = clamp(a, −a_max, a_max)          (skip if a_max is null)
v   = clamp(v + a · h, −v_max, v_max)
x   = x + v · h                         (then wrap if continuous)
```

This is semi-implicit Euler. With `a_max = null`, no latency and one substep, it is identical to `HeadSpring.step` in idea.md.

### 9.5 Ideal axes (dynamics null)

x := T clamped to the limits, v := diff(x, x_prev) / dt, a := 0.

### 9.6 Limits

If x is outside the limits it is clamped to them, and v and a are set to 0 if they point outwards.

### 9.7 Initial state

On the first update, each internal axis's x is the reported position if one was reported, otherwise rest; v = a = 0; T = x; engaged = false.

## 10. Output

`update` returns:

- `setpoints`: for each internal joint axis key (6.1), `{position, velocity, acceleration}` (the reference state after 9.6). Base yaw setpoints are in M.
- `diagnostics` (read-only, for logging and plots):
  - `gaze_error`: angle between the vector from the eye point to the target and the gaze direction, computed as atan2(|a × b|, a · b) for accuracy at small angles. The gaze direction is the facing of the furthest-out non-null part in the final pose table; for binocular eyes, the sum of the two eye directions. 0 when idle.
  - `reachable`: true if the raw target of the furthest-out non-null part, before clamping, was within its limits.
  - `lead`: per internal axis, diff(x, x_meas) before the governor.
  - `base_state`: `"hold"` or `"turn"`, or absent if the base is not internal.
  - `vergence`: angle between the two eye directions when binocular, else absent.

### 10.1 Logging

Implementations SHOULD log, through the platform's standard logging facility and at debug level, each change of: `reachable` (naming the joint and the angle it would need), the base state, and the governor holding a joint back or releasing it. They SHOULD log once, at warning level, when an external joint has not been reported before the first update. Messages are logged on changes only, never every update. Message wording is not normative and is not part of conformance.

Implementations MUST reject non-finite measurements and targets.

## 11. Conformance

Details are fixed in phase 3; this section sets the rules.

- A scenario is a JSON file: a settings document (inline or a path), `dt`, the number of steps, a target script (a list of `[t, x, y, z]` or `[t, null]` keyframes, held until the next key), and an optional measurement script for closed-loop tests.
- The reference implementation produces a golden trace: one JSON object per update (JSONL) with `t`, every setpoint and every diagnostic.
- A port passes if every value matches the golden trace within that port's tolerance, and every boolean and state string matches exactly. Tolerances (absolute, radians or seconds) are set per port, and per precision where a port offers both, when the port is built; they are recorded in this section.
- Required scenarios include an idea.md compatibility scenario: the `head_only` preset in open loop, with `max_substep` set to the frame time, must match idea.md's `head_share` and `HeadSpring` (with idea.md's 0.3° rest offset removed as in 8.1).

## 12. Versioning

The spec uses semantic versioning. Settings documents declare `spec_version`; an implementation MUST reject a document with a different major version, and SHOULD warn on a newer minor version.
