# lookaround: research notes

Literature review on adding upper-body (torso) yaw and whole-body (base) yaw to the eye-head model in [idea.md](idea.md). It feeds the design in [plan.md](plan.md).

Search done on 2026-10-09. Sources: Pejsa's thesis and Sidenmark & Gellersen 2019 were read in full; Mehrotra 2025 and Anastasopoulos 2015 from full-text summaries; the rest from abstracts or from citations in those papers.

## Summary

- No published model or package does continuous tracking across eyes, head, torso and base. The nearest work (Pejsa 2015) models eyes, head, torso and feet, but as discrete, pre-planned gaze shifts.
- Torso: in people it joins only past about 45° of gaze shift, standing, and mostly for horizontal shifts. It covers up to about 20% at 100°, starts about 300 ms after the head, and keeps moving after the eyes have landed.
- Seated, swivel chair and standing are measurably different: torso yaw SD is 11.5°, 47° and 56° respectively. Rotational freedom matters more than standing upright.
- Base (feet): turns last, in a fixed order (eyes → head → trunk → feet), with the feet about 250 ms after the torso. No paper gives a rule for when to turn the feet, so that threshold and dwell time are ours.
- Compensation: the neck corrects for whatever the trunk is doing, keeping the head-in-space trajectory the same (Land 2004). This supports computing each joint from its parent's *measured* angle.

## Prior art closest to this project

- Pejsa, Andrist, Gleicher, Mutlu (2015), "Gaze and Attention Management for Embodied Conversational Agents", ACM TiiS, written up in full in Pejsa's 2016 PhD thesis (*Effective Directed Gaze for Character Animation*, UW–Madison). This is the nearest existing solution: a parametric gaze-shift model for eyes + head + torso + whole body, with alignment parameters αH, αT and αB. These are "how far past the minimum each part goes", the same idea as our per-stage share. Key formulas (degrees, ms):
  - Minimum torso rotation for gaze amplitude A: `0` if A < 20; `0.08A − 1.56` if 20 ≤ A < 40; `0.43·exp(0.03A) + 0.19` if A ≥ 40 (fitted to McCluskey & Cullen 2007 monkey data; small: ≈9° at A = 100).
  - Torso latency `τ_T = 0.25A + 47.5` ms; feet latency `τ_B = τ_T + 250` ms.
  - Peak velocities: head `(4/3·A_H/50 + 2/5)·70 °/s`; torso `(4/3·A_T/15 + 2)·10 °/s`; eyes after Baloh 1975.
  - Standing: the pelvis takes about half the torso rotation. Torso yaw only, in the horizontal plane. A seated/standing flag.
  - The gap we fill: Pejsa is event-based. It plans discrete gaze shifts with velocity profiles and has no continuous tracking of a moving target. It has no robot or joint-limit or external-base modes, and no portable implementation. lookaround's cascade of springs is the continuous-tracking counterpart, so cite it as the main prior art.
- Andrist et al. 2012 (head-eye model, the head-only predecessor) and Pejsa et al. 2013 "Stylized and Performative Gaze" (non-human proportions, which matter for the owl and robot presets).
- The Greta ECA platform implements a threshold version (eyes only up to 60°, head > 60°, upper torso > 90°, lower torso > 120°), citing Pejsa and McCluskey & Cullen. A simple baseline to compare against.

## Human data for the torso and base split

- Sidenmark & Gellersen (2019), "Eye, Head and Torso Coordination During Gaze Shifts in Virtual Reality", ACM TOCHI 27(1). doi:10.1145/3361218. 7,600 gaze shifts, seated and standing, horizontal, vertical and diagonal. The best source for our torso numbers:
  - Up to 15° the eyes do it alone; up to 25° the eyes do > 90%. Three gaze zones: < 25° eyes, 25–50° eyes + head, > 50° eyes + head + torso.
  - No torso below about 45°. Standing, horizontal or diagonal, ≥ 45°: torso used in 85% of shifts, lagging the head by about 300 ms (550 ms on average overall). Torso contribution up to 20% at 100°. Almost no torso for vertical shifts or when seated.
  - Head contribution reaches about 60% at 100°. The head ends short of the target (consistent with the 0.69 share). Large individual differences (up to 30°).
  - Head and torso keep moving after the eyes have reached the target, towards a comfortable posture, while the eyes counter-rotate (VOR).
  - Customary head-on-torso range: 80–160° in total standing, 130–180° seated (seated uses more neck because the torso can't help).
  - For large shifts the eyes move in steps: they jump, wait for the head, then jump again. This is a useful check for the phase 2 saccade model.
- Mehrotra, Silver, Bischof, Kingstone (2025), "Sit, stand, and swivel", PLoS One 20(10): e0334182. 360° panoramas. Torso yaw SD: standing 56°, swivel chair 47°, fixed chair 11.5°. "Rotational freedom, not upright posture, drives naturalistic gaze." The head leads the torso by 34–53 ms in free exploration. This directly justifies the seated, swivel and standing presets.
- Land (2004), Exp Brain Res 159; and "The coordination of eyes, head and trunk in very large natural gaze saccades", J Vision 4(8). Kitchen (trunk free) vs driving (trunk fixed): head-in-space is the same, the neck compensates for the trunk, and a vestibulo-collic feedback model. This supports the cascade design.
- Anastasopoulos et al. (2015), "Fast gaze reorientations by combined movements of the eye, head, trunk and lower extremities", Exp Brain Res. doi:10.1007/s00221-015-4238-4 (PMC4369292). Targets at 45–180°. Eye-in-orbit saturates at about 47°. Head-on-trunk and trunk-in-space are linearly coupled, and the trunk moves *more* than the neck. PCA shows 3 mechanical DOF collapsing to about 2 kinematic DOF (90% / 10% variance). Leg muscles can activate *before* the eyes for predictable targets. This means a simple linear torso share, as planned, is a fair first approximation.
- McCluskey & Cullen (2007), "Eye, head, and body coordination during large gaze shifts in rhesus monkeys", J Neurophysiol 97(4):2976–2991. Torso contribution grows linearly with amplitude; the torso starts after the eyes and head; posture matters. The source of Pejsa's formulas. (Monkeys, not humans.)
- Hollands, Ziavra, Bronstein (2004), Exp Brain Res 154(2):261–266, and Hollands, Patla, Vickers (2002), "Look where you're going!", Exp Brain Res 143:221–230. Whole-body turns run in a fixed order: eyes → head → trunk → feet. During walking, the head lines up with the new travel direction before the body turns. Useful if a walking base is ever added.
- Hu, Sidenmark, Lee, Gellersen (CHI 2026), already in idea.md: soft-hinge (τ knee, s softness, β scale) eye-head model, restricted to ±50° *because* beyond that "amplitudes increasingly recruit the torso". So our torso hinge picks up exactly where their model stops. Using the same softplus hinge form for the torso keeps the model consistent.

## Robotics prior art

- Roncone, Pattacini, Metta, Natale (2016), "A Cartesian 6-DoF Gaze Controller for Humanoid Robots", RSS XII (iCub: 3-DoF neck + 3-DoF eyes). Resolves eye/neck redundancy by optimisation, with VOR-style stabilisation.
- Gaze stabilisation surveys (e.g. arXiv 1411.3525; "Motion Control on Bionic Eyes", arXiv 1901.01494) cover the vestibular side, should anyone mount this on a moving robot.

## What this changes in the plan

1. Torso: a softplus hinge, knee 45°, share ≈ 0.35, yaw only, so the torso is ≈20% of a 100° shift (Sidenmark). Pejsa's exponential minimum is an alternative curve; plot both in the testbed.
2. Presets: `human_standing`, `human_seated` (torso ±10°, base external), and `human_swivel` (torso wide or continuous, base external). All three have published data (Mehrotra 2025).
3. Optional per-stage onset latency (plan A8) and velocity scaling with amplitude (Pejsa, Guitton & Volle) in the dynamics.
4. Base turn: keep it a *decision* (body torque vs reorientation), with hand-picked thresholds, and say so in the docs.
5. Validation figures: overlay our curves on Sidenmark Fig. 9 and 10 (contribution vs amplitude, by posture) and on Pejsa's A_T,min. This is the core of the "science" story.
6. Positioning for the README and JOSS paper: "a *continuous*, real-time, portable generalisation of event-based gaze-shift models (Pejsa 2015) to N-segment chains, grounded in Kim 2010, Sidenmark 2019, Hu 2026 and Mehrotra 2025."

## References (additions to idea.md)

- Sidenmark, L., Gellersen, H. (2019). Eye, Head and Torso Coordination During Gaze Shifts in Virtual Reality. ACM TOCHI 27(1). https://doi.org/10.1145/3361218 · [PDF](https://eprints.lancs.ac.uk/id/eprint/136779/1/Eye_Head_and_Torso_Coordination_During_Gaze_Shifts_in_Virtual_Reality.pdf)
- Pejsa, T., Andrist, S., Gleicher, M., Mutlu, B. (2015). Gaze and Attention Management for Embodied Conversational Agents. ACM TiiS 5(1). · Pejsa, T. (2016). *Effective Directed Gaze for Character Animation*. PhD thesis, UW–Madison. [PDF](https://graphics.cs.wisc.edu/Papers/2016/Pej16/thesis-pejsa-final.pdf)
- Mehrotra, A., Silver, C., Bischof, W. F., Kingstone, A. (2025). Sit, stand, and swivel: Posture affects visual exploration of panoramic scenes in virtual reality. PLoS One 20(10): e0334182. https://doi.org/10.1371/journal.pone.0334182
- Anastasopoulos, D. et al. (2015). Fast gaze reorientations by combined movements of the eye, head, trunk and lower extremities. Exp Brain Res. https://doi.org/10.1007/s00221-015-4238-4 · [PMC4369292](https://pmc.ncbi.nlm.nih.gov/articles/PMC4369292/)
- Land, M. F. (2004). The coordination of rotations of the eyes, head and trunk in saccadic turns produced in natural situations. Exp Brain Res 159. Also: The coordination of eyes, head and trunk in very large natural gaze saccades. J Vision 4(8).
- McCluskey, M. K., Cullen, K. E. (2007). Eye, head, and body coordination during large gaze shifts in rhesus monkeys: Movement kinematics and the influence of posture. J Neurophysiol 97(4):2976–2991.
- Hollands, M. A., Ziavra, N. V., Bronstein, A. M. (2004). A new paradigm to investigate the roles of head and eye movements in the coordination of whole-body movements. Exp Brain Res 154(2):261–266.
- Hollands, M. A., Patla, A. E., Vickers, J. N. (2002). "Look where you're going!": gaze behaviour associated with maintaining and changing the direction of locomotion. Exp Brain Res 143:221–230.
- Roncone, A., Pattacini, U., Metta, G., Natale, L. (2016). A Cartesian 6-DoF Gaze Controller for Humanoid Robots. RSS XII. https://roboticsproceedings.org/rss12/p22.html
- Andrist, S., Pejsa, T., Mutlu, B., Gleicher, M. (2012). A Head-Eye Coordination Model for Animating Gaze Shifts of Virtual Characters. Gaze-In '12.
- Drillis, R., Contini, R. (1966). Body segment parameters. NYU Tech. Report 1166-03 (via Winter, *Biomechanics and Motor Control of Human Movement*).
- ROS REP 103: Standard Units of Measure and Coordinate Conventions.

Not yet checked: Imai, Moore, Raphan, Cohen (2001), Interaction of the body, head, and eyes during walking and turning, Exp Brain Res 136. This is relevant only if a walking base is added.
