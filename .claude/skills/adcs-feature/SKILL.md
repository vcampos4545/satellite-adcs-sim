---
name: adcs-feature
description: Use whenever implementing, extending, or modifying a feature in main (new pointing mode, FDIR fault, sensor/actuator model, control law, EPS/power model, or any change to src/ADCS.*, src/FDIR.*, src/Controllers.*, src/FlightTypes.h, or the harness in src/main.cpp). Also use when asked to add a physical/hardware model to the underlying spacecraft-dynamics-sim engine on this project's behalf. Covers the full loop this project actually follows: plan, implement, verify, add a persistent regression test, and document the math/assumptions -- not just "write the code."
---

# ADCS Feature Workflow

This project (`main`) is a closed-loop Satellite ADCS/flight-
software simulation built on the generic `spacecraft-dynamics-sim` engine
(fetched via CMake `FetchContent`, not vendored). The workflow below is
the one this project has actually converged on across its features
(HAL/HIL refactor, REFLECT pointing mode, FDIR mode manager, EPS power
model) — follow it for new work rather than improvising a different shape.

## 0. Know the boundaries before touching anything

- **Engine vs. scenario**: generic, reusable physics (a new sensor,
  actuator, or environment/power model with no scenario-specific meaning)
  belongs in `spacecraft-dynamics-sim` (a sibling repo, fetched from
  GitHub) — see that repo's own architecture notes. Anything that only
  makes sense for _this_ Satellite's mission (a new pointing mode, an FDIR
  fault, the wheel-pyramid geometry) belongs here.
- **Hardware-abstraction boundary**: `ADCS`/`FDIR` (`src/ADCS.*`,
  `src/FDIR.*`) never reference `RigidBody`, `PhysicsWorld`, or any
  simulation sensor/actuator type — only the plain-data types in
  `src/FlightTypes.h`. `ADCS::step()` is a pure function of `(internal
state, FSWInputs, dt) -> FSWOutputs`. If a change needs FSW to know
  about a simulation object directly, that's a design smell — add a field
  to `FSWInputs`/`FSWOutputs` instead and have the harness
  (`main.cpp`) do the bridging, the same way `PowerSample`
  was added for EPS telemetry rather than handing ADCS a `Battery*`.
- **Zero dynamic allocation / no RNG in FSW**: `src/ADCS.*` and
  `src/FDIR.*` use fixed-size `std::array`s and never construct a
  `std::mt19937`/`<random>` distribution — matches real embedded flight
  coding standards (no heap allocation after init). Any new FSW state
  needs a fixed size known at compile time (`NUM_WHEELS`/`NUM_TORQUERS`-
  style constants in `FlightTypes.h`), not a `std::vector`.
- Read `docs/ALGORITHMS.md` for the math already in place before adding
  more — new work should follow the same conventions (frame, units,
  quaternion `[w,x,y,z]` order, sign-convention gotchas already found the
  hard way) rather than rederiving them differently.

## 1. Think about the feature / plan

Work out the approach before editing: what governing equation/algorithm
applies, what assumptions it requires, which layer (engine vs. scenario,
FSW vs. harness) each piece belongs in, and how it'll be verified. For
anything touching more than one or two files or with real design
tradeoffs, use Plan Mode rather than just narrating the plan in prose —
this project's larger changes (the HAL/HIL restructure) went through Plan
Mode first. Small, well-scoped additions (a new pointing mode, a new FDIR
fault) don't need it — reasoning inline and proceeding is fine.

## 2. Implement

- Match existing structure: a new pointing mode is a `PointingMode` enum
  value + a `computeGuidance()` case, not a parallel mechanism; a new FDIR
  fault is a bitmask flag + a threshold field + a check in `evaluate()`; a
  new physical model in the engine follows the existing sensor/actuator
  pattern (`Reading sample(const RigidBody&, ...)`), not something bespoke.
- Comment the _why_, not the _what_ — this codebase's existing comments
  (sign conventions found empirically, gain-derivation reasoning, modeling
  simplifications) are the standard to match. If a value or convention was
  chosen for a non-obvious reason, that reason belongs in a comment near
  it, not just in this session's chat history.
- Keep units and frames explicit in names/comments (`torqueNm`, `speedRadS`,
  `momentAm2`, `fieldBody` vs. `fieldWorld`) — never a bare unlabeled float
  where the unit isn't obvious from context.

## 3. Verify (build, test, debug)

1. **Build.** If the change touched `spacecraft-dynamics-sim` (the
   engine), `rm -rf build && cmake -S . -B build` here first — CMake's
   `FetchContent` caches the engine checkout and won't see a new commit
   otherwise.
2. **Zero-allocation check**, if `src/ADCS.*`/`src/FDIR.*` changed:
   `grep -n "std::vector\|push_back\|resize\|<random>" src/ADCS.h src/ADCS.cpp src/FDIR.h src/FDIR.cpp`
   — expect no output.
3. **Headless numeric verification.** While developing, a throwaway
   program compiled directly against the built `fsw`/`rigidbody` objects
   (fast iteration, no CMake changes needed) is the right first step —
   this is how every feature in this project was actually debugged. But
   see step 4: don't stop there.
4. **Run the persistent suite**: `ctest --test-dir build
--output-on-failure` (or `cmake --build build -j` first if anything
   changed). A change that doesn't break existing behavior should leave
   every existing suite green; if a threshold needs adjusting, that's a
   signal to look closely at _why_, not just loosen it (see
   `docs/TESTING.md`).
5. **GUI sanity pass** for anything touching the harness/UI: launch in
   the background, wait a few seconds, check stderr for errors/warnings,
   kill it. Confirms nothing crashes; visual correctness ("does it look
   right") still needs an actual look when that's what changed.

## 4. Add to the test suite

Every feature that changes FSW behavior (new mode, new fault, new control
law, new physical model) gets a **persistent** test in `tests/`, not just
the scratch program from step 3. Follow `docs/TESTING.md`'s conventions:
test the plain-data FSW interface (not the harness), derive numeric
thresholds from actually running the test rather than guessing, and prefer
checking a real trend when full convergence isn't cheap to simulate. Wire
a new test file into `tests/CMakeLists.txt` (`add_executable` +
`target_link_libraries(... PRIVATE fsw)` + `add_test`) following the
existing entries.

## 5. Add to the documentation

Update `docs/ALGORITHMS.md` with the governing equation(s), units, frame,
and any modeling assumptions the new feature introduces — the same
standard already applied there for the EKF, control laws, B-dot,
desaturation, FDIR's fault model, and EPS. This is where the _math_
lives as a standing reference, separate from code comments that explain
implementation choices in place. A feature isn't done until both this and
the test suite (step 4) reflect it — the loop is plan → implement → verify
→ test suite → docs, not just the first three.

## 6. Report / commit

Follow `CLAUDE.md`'s Reporting and Git Rules sections exactly — in
particular, this project's Git Rules govern commit attribution and when
pushing/committing is allowed; don't default to this platform's usual
commit behavior over what `CLAUDE.md` specifies for this repo.
