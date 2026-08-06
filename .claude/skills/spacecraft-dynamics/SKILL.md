---
name: spacecraft-dynamics
description: Physics/math conventions and a verification checklist for touching attitude dynamics, estimation, or control code in satellite-adcs-sim (src/ADCS.*, src/Controllers.*, or attitude-related code in the spacecraft-dynamics-sim engine). Use alongside the adcs-feature skill's workflow when the change itself involves rigid-body/attitude math, not just wiring or UI.
---

# Spacecraft Dynamics Conventions

The full derivations already in place live in `docs/ALGORITHMS.md` — this
is the checklist to apply *while* touching that kind of code, not a
duplicate of it. Update `docs/ALGORITHMS.md` itself when the math changes
(see the `adcs-feature` skill's step 5).

## Frames

This project has **no orbital mechanics** — no inertial-to-orbit
transform, no LVLH frame, no orbital rate, no eclipse model. Only two
frames exist:

- **World frame**: the simulation's fixed global frame (`target`,
  `sunPosition`, `ambientFieldWorld`).
- **Body frame**: fixed to the spacecraft (sensor readings, actuator
  axes, the EKF state).

Don't introduce an orbital/LVLH frame or reference one in comments/docs —
if a future feature genuinely needs orbital mechanics, that's a
`constellation-sim`-scale addition (see this repo's README's "Scope"
section), not something to bolt on here.

## Attitude convention

Quaternions are `[w, x, y, z]` (`glm::quat`'s own scalar-first
convention) throughout. Body +Z is the payload/pointing axis every
guidance mode aims; body -Z is the star tracker boresight (deliberately
opposite, so it isn't staring at whatever +Z currently points toward).

## Rigid body dynamics

Euler's equation, principal-axis or full inertia tensor as applicable:

```
I*omega_dot + omega x (I*omega) = tau
```

`ADCS`/`Controllers` never integrate this themselves — that's the
engine's (`RigidBody`/`PhysicsWorld`) job. FSW only ever computes a
commanded torque/dipole moment; verifying a *control law* means checking
what torque it commands given a state, not re-deriving the plant.

## Sign conventions (check this specifically — found wrong before)

`ReactionWheel`'s simulated reaction dynamics apply the *negative* of
whatever torque is commanded (Newton's-third-law reaction). Every
hand-derived law that doesn't go through the normal actuator allocator
(detumble's rate-damping law, the cross-product desaturation law) has to
bake in a compensating sign flip. Getting this backwards doesn't cause a
build error or an obviously-wrong small deviation — it's stable-looking
right up until it pumps momentum the wrong way or destabilizes pointing
at high saturation. If a new hand-derived law reads suspiciously like a
textbook formula copied verbatim, verify its sign empirically (see below)
rather than trusting the derivation alone — this has bitten this project
twice already (`CascadedController`'s original gain sign, and the
desaturation law's sign).

## Verification checklist

For any change to attitude propagation, estimation, or control:

- **Quaternion normalization**: every place a quaternion is updated
  incrementally (strapdown integration, a multiplicative EKF reset)
  re-normalizes immediately after.
- **Units and frames**: every vector's frame (world vs. body) and unit
  (rad vs. rad/s vs. N·m vs. A·m²) should be unambiguous from its name or
  an adjacent comment — see `docs/ALGORITHMS.md`'s convention note.
- **Numerical sanity via headless simulation, not just inspection**: step
  the closed loop (control law + a hand-integrated plant, or a real
  `RigidBody`) for a physically meaningful duration and check the
  quantity that should move actually moves the right direction/converges
  — this is how B-dot's, the cascaded controller's, and the desaturation
  law's sign bugs were actually caught, not by reading the derivation
  again. See `tests/test_adcs_control.cpp` for the pattern.
- **Compare against the textbook form** when a law comes from a known
  reference (TRIAD, B-dot, cross-product-law desaturation, the EKF
  propagate/correct equations) — note in a comment *where* it diverges
  from the reference and why (usually: this project's specific sign
  convention, or a simplification with its own documented assumption).
