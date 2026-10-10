# lookaround: plan

Status: draft 2 (2026-10-09). Done: the spec (docs/SPEC.md), schema and presets; the Python reference library (python/, 45 tests, including the idea.md compatibility test); a basic testbed A (WebSocket server + browser mannequin). The source model is in [idea.md](idea.md); the literature review is in [research_notes.md](research_notes.md).

## Goal

`import lookaround` gives any "thing that can look around" (a person, a robot head, a turret, an owl) natural-looking gaze behaviour. You describe its body as a chain of parts in a settings file. Each tick, the application reports where the parts actually are and where the target is, and lookaround returns where the parts it controls should go next. Those setpoints feed the application's own PID loops, servos or animation.

The idea.md model covers eyes and head. This project adds:

- Upper-body swivel (torso yaw).
- Whole-body re-orientation (base yaw: a person turning on their feet, or a robot base rotating).

Any part can be missing, limited, unlimited, driven by lookaround, or driven by something else.

Python is the reference implementation. C++ (header-only), Rust, C# and TypeScript ports follow and are held to it by a shared conformance suite. A ROS 2 package and a real robot head are the hardware target.

Deliverables, in order of importance:

1. A language-neutral spec (`docs/spec.md`), a JSON Schema and shared settings presets. Drafted 2026-10-09 as spec 0.1.0-draft.
2. The Python reference library.
3. Two testbeds: an animation testbed (open loop: a person looking at a moving ball, a head on its own tracking a ball in front of it) and a robot simulation (closed loop: simulated actuators with PID, lag and disturbances).
4. A conformance suite (golden traces) that every port must pass.
5. A ROS 2 package, then the real robot head.
6. The other ports and a live web demo.

---

## Decisions

| # | Topic | Decision |
|---|---|---|
| D1 | Units | Radians and SI units in the API and all ports, with ROS REP 103 axes (x forward, y left, z up). Degree helpers are provided; docs and tables stay in degrees for readability. |
| D2 | Torso and base axes | Yaw only (turning). No torso lean. The data types leave room for pitch and roll later. |
| D3 | Settings | One JSON file. Each part (`eyes`, `head`, `torso`, `base`) is either `null` (absent) or a params object, with `"external": true` for parts driven by something else (default false). See A1. |
| D4 | Hardware | Yes: the user's own robot head, in a later phase. Hence acceleration limits, velocity feed-forward in the outputs, closed-loop design, and ROS 2 / ros2_control. |
| D5 | Eyes | A `binocular` boolean in the eyes settings (with `ipd`). False gives one cyclopean gaze; true gives a left and right eye that converge on near targets. |
| D6 | Licence | MIT OR Apache-2.0 (dual). This is the Rust convention and works on PyPI, npm and NuGet too. |
| D7 | Names | Checked 2026-10-09: `lookaround` is free on PyPI, npm and NuGet. On crates.io it is taken (an unrelated LAN address tool, v0.1.7). The Rust crate will be `lookaround-gaze` (imported as `lookaround_gaze`); everything else is `lookaround`. Publish real early versions rather than placeholders. |
| D8 | Target input | A point relative to the system (in the base frame), not a world point. The application (or tf2 in ROS) does the world-to-base transform. |
| D9 | ROS | ROS 2 is a planned phase, not optional. |
| D10 | Platform | The testbed runs natively on Windows (MuJoCo, Rerun). ROS 2 runs in WSL2 with Ubuntu 24.04. |

---

## Design

### A1. Settings: params or `null` per part, three states

```json
{
  "name": "human_standing",
  "eyes":  { "binocular": false, "ipd": 0.063,
             "yaw_limit": 0.61, "pitch_limit": [-0.61, 0.61], "...": "..." },
  "head":  { "offset": { "xyz": [0, 0, 0.11], "rpy": [0, 0, 0] },
             "yaw": { "limit": [-1.4, 1.4], "share": 0.69 }, "pitch": { "...": "..." },
             "dynamics": { "omega": 5.0, "v_max": 1.57, "a_max": 12.0, "latency": 0.0 } },
  "torso": { "yaw": { "limit": [-0.79, 0.79], "knee": 0.79, "share": 0.35 }, "...": "..." },
  "base":  { "external": true }
}
```

| State | Meaning | lookaround reads | lookaround outputs |
|---|---|---|---|
| `null` | The part doesn't exist and is fixed at its rest pose. | nothing | nothing |
| params, `external: false` (default) | lookaround plans this part's motion. | its measured angle (and velocity, if known) | a setpoint: position, velocity feed-forward, acceleration |
| params, `external: true` | Something else moves this part: a seated person's chair, a nav stack's base, a balance controller. | its measured angle | nothing |

There is no "please turn the body" request. An external part is just another input.

### A2. The control loop: report the pose, query the setpoints

```python
import lookaround as la

looker = la.Looker.from_file("presets/human_standing.json")   # or from_dict(...)

while running:
    looker.report(measured)                  # measured joint angles (+ velocities) of every non-null part
    sp = looker.update(dt, target=ball_in_base_frame)   # target=None -> idle
    for joint, s in sp.items():              # only internal parts appear here
        pid[joint].setpoint(s.position, s.velocity)     # the app's PID / servo / animation
```

- Closed loop (robots). lookaround never assumes its setpoints were reached. Every target is computed from the measured pose.
- Open loop (animation, conformance tests). If nothing is reported, the measured pose is taken to be lookaround's own last setpoint. This is the same code path with perfect actuators.
- The PID is not in lookaround. It lives in the application: ros2_control, MuJoCo actuators, servo firmware or the game engine. lookaround supplies smooth, limited reference trajectories with feed-forward.

### A3. Compensation falls out of the cascade

Each part's target is worked out relative to its parent's measured angle, from the base out to the eyes:

```
target relative to base            -> base reference   (internal only; hysteresis + dwell, see A6)
target relative to measured torso  -> torso reference  (softplus hinge: joins only for large shifts)
target relative to measured head   -> head reference   (idea.md head_share: 0.69 yaw, 0.52 up, hinge down)
target relative to measured head   -> eyes = remainder, clamped
```

So if a part doesn't get where it was asked to go, the parts further out make up the difference automatically. That covers an external part, a slow or lagging actuator, a part held back by a balance controller with other priorities, or a part sitting on a joint limit. This is the vestibulo-ocular reflex (eyes counter-rotate as the head moves) and the vestibulo-collic reflex (the neck compensates for the trunk; Land 2004) as a single mechanism, and the same idea as idea.md's "eyes = gaze − current head", applied all the way down the chain.

### A4. Model–robot delay and mismatch

A physical robot lags the model (servo response, bus latency, PID settling). Three mechanisms, all specified in the spec and set per part in the settings:

1. Reference governor (anti-windup). Each internal part's reference spring is not allowed to run more than `max_lead` (rad) ahead of the measured angle. Past that, the reference is pulled back towards the measured angle. This stops the plan running away from a robot that can't keep up, and stops it overshooting when the robot catches up. It's the gaze equivalent of integrator anti-windup.
2. Latency compensation. Given a known actuator latency `L` for a parent part, its measured angle is extrapolated by `L · measured_velocity` before the child's target is computed, like a light Smith predictor. Off by default.
3. Reported velocity is optional. If the application doesn't report velocity, lookaround estimates it from successive reports with a simple filter, written into the spec so that every port matches.

Testbed B has to show all three working (scenarios B2 and B3).

### A5. Geometry: the full pose of every part

`height` alone is not enough. Each part in the settings has a fixed offset from its parent's pivot (`xyz` + `rpy`, as in a URDF joint origin). The runtime pose is the reported joint angles. Together these give the eye position at every tick, which is needed for near targets: a ball 40 cm from a face shows parallax as the head and torso turn. With `binocular: true` it also gives each eye's position.

- Human presets can be generated from height using standard anthropometric ratios (Drillis & Contini 1966: eye ≈ 0.936 H, shoulder ≈ 0.818 H, hip ≈ 0.530 H), via a helper rather than as part of the core.
- A URDF importer (ROS phase) fills in the offsets from the robot description, so the robot head never needs hand-entered geometry.
- The target is a point in the base frame (D8). In ROS, tf2 supplies it directly with `lookupTransform(base_link, ball)`.

### A6. Base: a decision, not a share (internal base only)

People don't turn their feet for a glance. Turns go eyes → head → upper body → feet (Hollands et al. 2004; Anastasopoulos et al. 2015), with the feet about 250 ms after the torso (Pejsa 2015). Rule, with hand-picked values (no paper gives one; see research_notes.md):

- If the target is beyond what head + torso + eyes can reach comfortably (about 100°), turn now.
- Otherwise, if the target has stayed more than about 30° off the base heading for longer than about 1.5 s, turn to face it.
- Turn with a slower spring, and let the cascade re-centre the torso, head and eyes.

Because the target is given relative to the base, the base setpoint is a yaw relative to the current heading, plus a yaw rate.

### A7. Human yaw allocation (v1 presets)

| Part | Joins when | Share | Hard limit | Source |
|---|---|---|---|---|
| Eyes | always | remainder | ±35° (preferred ±~20°) | idea.md; Stahl 1999; Sidenmark 2019 |
| Head | always | 0.69 | ±80° | Kim et al. 2010; Sidenmark 2019 (customary head range) |
| Torso (standing) | gaze relative to torso > 45° (soft knee) | ≈0.35 beyond the knee, which gives ≈20° at 100° | ±45° | Sidenmark & Gellersen 2019; Pejsa 2015 |
| Torso (seated) | only at the largest amplitudes | small | ±10° | Mehrotra 2025 |
| Torso (swivel chair) | like standing | like standing | wide | Mehrotra 2025 |
| Base | see A6 | rotates to face | none (wraps) | hand-picked |

### A8. Onset latency and dynamics

- An optional per-part `latency`: a "sticky" target that is only re-read once the error has persisted for τ. That needs no buffer, so it ports easily. Data: head after eyes 30–150 ms, torso after head 34–550 ms depending on task, feet after torso about 250 ms (research_notes.md).
- Critically damped spring with v and a limits (servos need the acceleration limit), integrated by semi-implicit Euler with variable `dt`, substepping when `dt` > 1/120 s.
- Optional velocity scaling with amplitude (main sequence; Pejsa / Guitton & Volle).
- Continuous and limited joints: a 360° head takes the shortest path; a head limited to ±170° may have to go the long way round. Limited joints are tracked as unwrapped angles.
- Eyes, phase 2: saccades (main sequence) + smooth pursuit with a speed cap + catch-up saccades. For large shifts the eyes step and wait for the head (Sidenmark 2019), which is a good visual check. Vergence when `binocular`.

### A9. Core shape

- Pure and tiny: no dependencies, no allocation in `update`, no I/O. Effectively `(state, settings, measured, target, dt) -> (state, setpoints, diagnostics)`.
- Diagnostics (read-only, no requests): gaze error, a `reachable` flag, per-part reference-minus-measured.
- Backward-compatibility anchor: a head-only open-loop configuration must reproduce idea.md's `head_share` / `HeadSpring` outputs. That's the first golden test.

### A10. Frames

- Chain: `base(yaw) -> torso(yaw) -> head(yaw, pitch) -> eyes(yaw, pitch)[x2 if binocular]`. Each angle is relative to its parent; offsets come from A5.
- yaw: + is left (counter-clockwise from above), wraps at ±π for continuous joints. pitch: + is up.
- Rest pose: everything at 0 except head pitch, REST_PITCH = −8°.

---

## Testbeds

There are two testbeds, because they answer different questions. Both read the same scenario files (ball paths, durations, seeds), so any scene can be run in either, and the robot's result can be compared with the ideal animation of the same scene.

### Testbed A: animation (open loop)

Question: does it look human? lookaround's setpoints are applied directly as the pose. There is no feedback and no physics.

Testbed A and the live web demo are the same 3D scene, built once in the browser (three.js). It has two halves:

- A live WebSocket bridge (built: `testbed/animation/server.py` + `web/testbed/`). Python runs the reference implementation at 60 Hz and streams each tick's pose to the page; the page sends back preset and path choices and the dragged ball position. This makes testbed A interactive from phase 2, long before the TS port.
- A Python batch runner. It runs scenarios through the reference implementation and writes traces in the conformance trace format (JSONL, spec section 11), plus the science plots. It needs no viewer.
- The web viewer. It shows the live bridge, and plays back those traces for the hosted (server-less) demo. After the TS port exists (phase 5), the same page runs lookaround live in the browser and becomes the public demo. Because the trace format is shared, the viewer can also show traces from any port, or from testbed B runs, next to the animation ideal.

| # | Scene | What it exercises |
|---|---|---|
| A1 | Head only, ball in front: Lissajous path, then random jumps | split, spring, eye remainder, saccades (spec 0.2), parallax, vergence |
| A2 | Standing person, ball orbits 360° at varying heights (floor to overhead) | torso recruitment, base dwell and turn, down-hinge |
| A3 | Bouncing ball (scripted ballistic path with bounces) passing around and behind the person | fast targets, reachability |
| A4 | Side by side: standing, seated, swivel and owl all watching the same ball | the generality pitch; main demo video |
| A5 | Ball teleports at random amplitudes, 1000 trials | science plots |

Ready-made pieces for testbed A (no need to build a character):

- VRM avatars loaded with three-vrm (pixiv). VRM fixes a humanoid skeleton including `hips`, `spine`, `chest`, `upperChest`, `neck`, `head`, `leftEye` and `rightEye`, so the hookup is a bone mapping. Base yaw goes on the root (or `hips`). Torso yaw is spread over `hips`, `spine` and `chest` (Pejsa: the pelvis takes about half when standing). Head yaw and pitch are spread over `neck` and `head`. Eyes go on `leftEye` and `rightEye`. three-vrm's own `vrm.lookAt` is switched off. Any VRM file works, so visitors can drop in their own avatar.
- Characters (the first option listed is the recommendation): the stylised capsule mannequin built in three.js (no assets, engineered look); a realistic human from MakeHuman / MPFB (exports are CC0; check that the rig has eye bones), exported as glTF or converted to VRM; VRoid Studio / VRoid Hub avatars (free, ready-made, but anime-styled). SMPL-X has eye joints but a non-commercial licence, so it is ruled out for a public demo.
- UI: Tweakpane for sliders bound to the settings, uPlot for live charts (see "Live web demo").

Outputs:
- Head contribution against gaze amplitude (A5), overlaid on Kim et al. 2010.
- Torso contribution against amplitude, overlaid on Sidenmark 2019 Fig. 9/10 and Pejsa's A_T,min curve.
- Main-sequence plots (peak velocity against amplitude) for the head and eyes.
- The demo video and the web demo.
- A parameter-provenance table in the docs: cited vs hand-picked.

### Testbed B: robot simulation (closed loop)

Question: does it work on hardware? Simulated actuators with PID loops, lag, saturation and disturbances; the measured pose is fed back through `report`. Bodies: `pan_tilt`, the user's robot head, and later a mannequin with actuators.

| # | Scene | What it exercises |
|---|---|---|
| B1 | Scenes A1 and A3 on `pan_tilt` and the robot head, MuJoCo actuators with PID | baseline tracking, acceleration limits |
| B2 | Sweep of actuator lag and PID gains | governor (`max_lead`), latency compensation, recommended PID bandwidth relative to `omega` |
| B3 | Disturbances: a joint held by a "balance" controller, a joint at its limit, an external part moved by hand | compensation by the parts further out (A3 in the design) |
| B4 | Same scenes in ROS 2 (Gazebo + ros2_control) | the ROS integration |
| B5 | Real robot head, same launch files with `use_sim:=false`; camera + ball detector | sim-to-real |

Ready-made pieces for testbed B:

- The MuJoCo interactive viewer (`mujoco.viewer`): drag bodies such as the ball with the mouse; the controller hooks in through a passive-viewer loop.
- Reachy Mini (Pollen Robotics / Hugging Face): an open-source desktop robot with a head and a rotating body (base yaw + head, a direct match for our chain). It ships a MuJoCo simulation (`reachy-mini-daemon --sim`) and a Python SDK. A ready-made B1 target, and a real robot that can be bought.
- MuJoCo Menagerie models: Robotis OP3 has head pan/tilt joints; other humanoids (Unitree G1, Booster T1) for later whole-body tests.
- ROS 2: `ros2_control_demos` and `gz_ros2_control` demos as the starting point for B4.

Outputs:
- Gaze error and lead (plan minus measured) over time, with and without compensation, across lag and gain ranges.
- Robot against animation: the same scenario in testbed A and B overlaid, showing how far the robot strays from the ideal.
- Tuning guidance for the docs (governor and PID settings for a given servo).

### Stack (the first option listed is the recommendation)

| Need | Testbed | Options | Notes |
|---|---|---|---|
| 3D viewing and data logging | B (A for debugging) | Rerun, meshcat, Open3D | Logs 3D and time series together. Python, C++ and Rust SDKs, so the ports can log too. |
| Viewer | A | the web demo page (Vite + three.js) playing back JSONL traces; Rerun as a quick debug view | One scene for testbed A and the demo; no physics engine. |
| Video | A | the viewer page driven frame by frame with Playwright → ffmpeg (WSL); Rerun screen capture | MP4/GIF for the README, rendered from the same scene as the demo. |
| Physics and actuators | B | MuJoCo (position/velocity actuators, your own PID on top), PyBullet, Genesis | Fast, Python, Windows. |
| ROS sim | B | Gazebo (Harmonic, paired with Jazzy) + `gz_ros2_control`; MuJoCo via `mujoco_ros2_control` | Gazebo is the standard ROS path and carries over to the real robot. |
| Robot description | B (and A for rendering) | URDF/xacro, MJCF generated from it | One source of truth for geometry; also feeds the URDF importer (A5 in the design). |
| Body model | A and B | stylised capsule mannequin / robot | Clean and engineered-looking; avoids SMPL licence issues. |
| Web demo | A (it is testbed A's viewer), plus a lag panel from B | three.js + the TS port + MuJoCo WASM; Babylon.js; React Three Fiber | See "Live web demo" below. |
| Figures | A and B | matplotlib + Jupyter; Plotly | Docs and paper. |

---

## Live web demo

Yes, this is possible, and it can be fully static: everything runs in the visitor's browser, hosted on GitHub Pages, with no server.

### What it does

- A 3D scene with the mannequin, the robot head and the other presets. Drag the ball with the mouse, or pick an automatic path (orbit, Lissajous, bounce, teleport).
- Preset switcher (standing, seated, swivel, owl, pan-tilt, robot head), plus a side-by-side mode where every preset watches the same ball.
- A switch for each part: absent / internal / external. Turn the torso off and watch the head and eyes take up the slack.
- Closed-loop panel: actuator lag, PID gains and a "hold the torso" disturbance, with the governor and latency compensation on or off. This is testbed B, live.
- Live charts of joint angles, reference minus measured, and gaze error.
- Sliders bound to the settings JSON, with export: tune a preset in the browser and download the JSON that the robot head loads.

### Stack (the first option listed is the recommendation)

| Need | Options | Notes |
|---|---|---|
| Build | Vite + TypeScript | Fast dev server, static output. |
| 3D | three.js; Babylon.js; React Three Fiber | three.js is the most widely known and has the smallest learning curve. |
| Gaze model | the TS port (`lookaround` on npm) | The demo uses the published package, so it doubles as an integration test of the port. |
| Physics | `@mujoco/mujoco` (DeepMind's official WebAssembly build); kinematic-only fallback | Loads the same MJCF scenes as the Python testbed, so ball physics and PID actuators behave the same as in testbed B. The WASM file is several MB; the kinematic mode keeps phones fast. |
| Controls | Tweakpane; lil-gui | Sliders generated from the settings JSON Schema. |
| Charts | uPlot; Chart.js | uPlot handles live streaming at 60 Hz cheaply. |
| Hosting | GitHub Pages via GitHub Actions; Netlify; Cloudflare Pages | Built and deployed from `web/` on each release tag. |

### Getting a demo online before the TS port exists

The port comes in phase 5, which is late for the first public demo. Because the demo page is also testbed A's viewer, it exists from phase 2:

1. Phase 2: locally, the page is fully interactive through the Python WebSocket bridge. The hosted copy on GitHub Pages has no Python server, so it plays back JSONL traces from the batch runner (scrub, rotate, switch presets and scenarios, charts).
2. Only if a goal needs it (for example, the hosted demo must be interactive before the TS port exists): run the Python reference in the hosted page with Pyodide (around 10 MB to load). Not worth doing for its own sake.
3. Phase 5: the TS port makes the page fully live: drag the ball, the closed-loop panel.

---

## ROS 2 integration (planned)

- Distro: ROS 2 Jazzy (LTS, supported to 2029) on Ubuntu 24.04 in WSL2. Rolling and Kilted are alternatives but not LTS.
- Package `lookaround_ros`:
  - Subscribes to `/joint_states` (the measured pose, fed into `report`).
  - Gets the target as `geometry_msgs/PointStamped` in any frame, transformed by tf2 into `base_link`.
  - Publishes setpoints to ros2_control: a `JointTrajectoryController` (position + velocity), or forward position/velocity controllers. The PID gains live in ros2_control.
  - Parameters: the settings JSON path, or ROS params mirroring it. The URDF importer fills in the A5 offsets.
  - Diagnostics on a topic for RViz / rqt_plot.
- Language: start with rclpy (wraps the Python reference). Move to rclcpp on the C++ header port.
- Real robot head: a ros2_control hardware interface for its servos (for example `dynamixel_hardware` if they're Dynamixels), the same URDF, and the same launch file with `use_sim:=false`.
- Extras: a launch file for each testbed B scene, a camera plus a ball detector (OpenCV) to close the loop on real vision.

---

## Engineering stack (the first option listed is the recommendation)

| Area | Choice | Alternatives |
|---|---|---|
| Repo | monorepo, one tag per spec version | one repo per port |
| Python | uv, ruff, pyright (strict), pytest, hypothesis (limits never exceeded, continuity, monotone split, wrap correctness, compensation invariants) | poetry, mypy |
| C++ | header-only C++17, `template<typename Real>`, no heap, CMake + doctest | C++20, Catch2 |
| Rust | `no_std` + `libm`, `serde` feature for settings, proptest; crate `lookaround-gaze` | optional PyO3 bindings |
| C# | netstandard2.1 (Unity + .NET 8), xUnit, NuGet | net8-only |
| TS | TypeScript, zero-dependency ESM, vitest, npm | plain JS |
| Settings | JSON + JSON Schema (validated in CI) | YAML, TOML |
| Conformance | scenario JSON → golden JSONL from Python → a runner per port. Tolerances are set per port when that port is built. Scenarios include closed-loop ones with scripted "measured" inputs | CSV |
| CI | GitHub Actions: 5 languages × Win/Linux, conformance, a ROS build job (`ros-tooling/setup-ros`) | |
| Docs | MkDocs Material | Sphinx, Docusaurus |
| Write-up | JOSS paper | arXiv tech note |

### Repo layout

```
lookaround/
  presets/*.json             # human_standing, human_seated, human_swivel, head_only, owl, pan_tilt, robot_head
  schema/settings.schema.json
  conformance/
    scenarios/*.json         # scripted targets, measured-pose scripts, dt, seed
    golden/*.jsonl           # generated by python, committed
  python/                    # reference: src/lookaround, tests/
  scenarios/                 # shared ball paths, used by both testbeds
  testbed/animation/         # testbed A: python batch runner, traces, science notebooks (the viewer is web/)
  testbed/robot/             # testbed B: mujoco scenes, PID, lag and disturbance sims
  ros/lookaround_ros/        # ROS 2 package (rclpy first), urdf/, launch/, config/
  ports/cpp/ ports/rust/ ports/csharp/ ports/ts/
  web/                       # testbed A viewer and live demo: vite + three.js (+ ports/ts, mujoco-wasm later), GitHub Pages
  docs/spec.md               # the spec: normative, language-neutral
  docs/
  .github/workflows/
```

---

## Phases

### Phase 0: Spec and skeleton
- `git init`, the repo layout, LICENSE-MIT + LICENSE-APACHE.
- Spec (`docs/spec.md`, drafted): frames, units, settings schema (A1), the report/update contract (A2), the cascade (A3), the governor and latency compensation (A4), geometry (A5), the base decision (A6), the integrator and velocity estimator.
- Presets + schema; parameter-provenance table.
- Read the full Land 2004 and McCluskey & Cullen 2007 papers; find cervical and thoracolumbar range-of-motion sources for the hard limits.
- Done when: the spec is reviewed.

### Phase 1: Python reference core
- The cascade, spring with v/a limits, governor, latency compensation, base decision, null/internal/external, open and closed loop.
- Tests: unit tests, hypothesis properties, and the idea.md backward-compatibility golden.
- Done when: the head-only config matches idea.md and all property tests pass.

### Phase 2: Testbeds
- Testbed A: the Python batch runner and science plots; the web viewer (Vite + three.js) playing back its traces, deployed to GitHub Pages as the first public demo; MP4 renders from the viewer.
- Testbed B: B1–B3 in MuJoCo: PID actuators, lag, disturbances; tune the governor.
- Eye dynamics: saccades, pursuit, binocular vergence.
- Done when: the demo video is rendered, the figures are in the docs, and B3 shows compensation.

### Phase 3: Conformance suite
- Scenario format (including scripted measured poses), golden generator, runner contract (stdin → stdout JSONL). Tag `spec-v0.1`.

### Phase 4: ROS 2 + sim
- URDF/xacro for a mannequin and for the robot head; `lookaround_ros` on rclpy; Gazebo + ros2_control; B4 in sim; RViz.

### Phase 5: Ports (suggested order)
1. C++ header: rclcpp node and the robot head.
2. TypeScript, then the full live web demo (three.js + MuJoCo WASM, all the features above), deployed to GitHub Pages.
3. Rust (`lookaround-gaze`), then C#.
- Each port is done when it passes conformance in CI.

### Phase 6: Real robot head and polish
- A ros2_control hardware interface for the head's servos; the same launch file in sim and real; a camera + ball detector.
- Web demo polish (preset export), docs site, JOSS paper.

---

## Risks

| Risk | Mitigation |
|---|---|
| Hand-picked torso and base numbers look wrong | They live in presets, are tuned in the testbed, and are flagged as hand-picked in the docs. |
| The closed loop oscillates (lookaround and the PID fighting each other) | The governor (A4), feed-forward velocity, and testbed B sweeps across lag and gain ranges (B2). Document recommended PID bandwidth relative to `omega`. |
| Ports drift apart | Spec + golden traces + CI conformance; ports target tagged spec versions only. |
| Float32 ports diverge over long runs | Per-port tolerances (set when each port is built), per-step comparison, short scenarios. |
| ROS learning curve, WSL quirks | ROS comes after the Python testbed works, so ROS problems never block the core. |
| Scope (5 languages + sim + ROS + hardware + web) | Phases 0–3 are the real project; every later phase ships on its own. |

## Deferred (not now)

- Unity sample scene (the C# port still targets netstandard2.1 so Unity can use it).
- Pyodide, unless another goal needs it (see "Live web demo").

## Open questions

1. The robot head: which servos (Dynamixel, hobby PWM, steppers, something else), what degrees of freedom (pan/tilt? eyes?), and what computer drives it (Pi, Jetson, PC, microcontroller)? This decides the ros2_control hardware interface and whether the C++ port has to run on a microcontroller.
2. ROS sim: Gazebo (the ROS standard) or MuJoCo inside ROS (reuses the testbed models)? I recommend Gazebo for the ROS phase and MuJoCo for the fast testbed.
3. Diagnostics: are a read-only `reachable` flag and gaze error OK as outputs? They're for logging and plots only, not requests.
