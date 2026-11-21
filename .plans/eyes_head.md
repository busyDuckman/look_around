Random notes from another proj:

A dead zone where only the eyes move exists only for looking down. Sideways and upward gaze shifts are split between eyes and head from the first degree:

Direction	Model	Values	Source
Yaw (sideways)	Head takes a fixed share	HMCR_YAW = 0.69	Kim, Reed & Martin 2010
Pitch up (above rest)	Head takes a fixed share	HMCR_UP = 0.52	Kim, Reed & Martin 2010
Pitch down (below rest)	Soft hinge: eyes alone first, then head takes all of the rest	DOWN_KNEE = 10 deg, DOWN_SOFT = 4 deg (width of the knee), DOWN_SHARE = 1.0	Hu et al. CHI 2026, Fang et al. 2015, Mehrotra et al. 2025, Stahl 1999
All directions	Eyes max out, head covers the rest	EYE_MAX = 35 deg	
Pitch is measured from the rest pose, REST_PITCH = -8 deg, which looks slightly down. The docstring also notes that Kim's 8% head share for downward targets was thrown out. It came from seated subjects making brief glances, and gave far too little head movement for a standing person staring at something low.

Fixational movements. Microsaccades, drift and tremor while holding a fixation. These are tiny, well under 1 deg.
Compensation. When the head moves, the eyes counter-rotate (the vestibulo-ocular reflex), so the gaze stays put while the head catches up. On a big shift the eyes get there first and the head arrives later.
What the model does

For sideways and upward shifts, the head takes a fixed share (69% / 52%) from the first degree, with no eye-only dead zone. That's Kim et al.'s average over fairly large, deliberate gaze shifts. For small glances it overstates head movement, since a real person would often just move their eyes.