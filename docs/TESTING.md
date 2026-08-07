# Testing

## What lives in `tests/`

Persistent, committed regression tests — one executable per feature area
(`test_adcs_control.cpp`, `test_fdir.cpp`, `test_eps.cpp`, ...), registered
with `ctest` via `tests/CMakeLists.txt`. Run all of them with:

```bash
cmake --build build -j
ctest --test-dir build --output-on-failure
```

or run a single suite directly (useful while iterating — each prints its
own `[PASS]`/`[FAIL]` lines with the actual numbers, not just a final
verdict):

```bash
./build/tests/test_adcs_control
```

No GoogleTest/Catch2 dependency — `tests/test_common.h`'s `CHECK()` macro
and a per-file `main()` are enough for what this project needs, matching
the "only pull in what's actually needed" convention the engine repo's own
`CMakeLists.txt` follows for its own dependencies.

## When a feature needs a test here vs. a scratch check

Every new feature that touches FSW logic (a new pointing mode, a new FDIR
fault, a new control law, a new physical model like EPS) should leave
behind a **persistent** test in `tests/`, not just a one-off `/tmp` program
compiled by hand and thrown away after confirming it works once. The
scratch-program style is still the right *first* step while developing —
fast iteration, no build-system changes needed — but once the behavior is
correct, promote the check into `tests/` before considering the feature
done. If it was worth verifying once, it's worth re-verifying automatically
every time something else changes nearby.

What "belongs in `tests/`" looks like, concretely:

- **Direction/trend, not just a hardcoded final number**, when the real
  physics doesn't converge within a duration that's cheap to simulate (see
  `test_adcs_control.cpp`'s B-dot/desaturation tests — a weak LEO field
  against even this small bus takes far longer than a test should run to
  fully converge; the test checks the trend is genuinely downward instead).
- **The specific numeric thresholds should come from actually running the
  test**, not from copying a plausible-looking number. If you change
  hardware parameters (wheel inertia, max torque, bus inertia, ...), the
  old thresholds may no longer hold even though the underlying law is
  still correct — rerun, look at the real margin, and adjust deliberately
  rather than loosening a threshold until it happens to pass.
- **Test the plain-data FSW interface** (`FSWInputs -> FSWOutputs`, or
  `FdirInputs -> PointingMode`), not the simulation harness — this is what
  `tests/test_common.h`'s `makeTestHardwareConfig()` is for. A test that
  needs `satellite_adcs_sim.cpp`'s GUI/harness to build is testing the
  wrong layer; if it's provably true of `fsw` alone, it should be checked
  against `fsw` alone (see `tests/CMakeLists.txt`'s comment on why FSW
  tests only need to link `fsw`, not `rigidbody`/`vgl`/`imgui`).

## What still only gets a manual/GUI check

A handful of things genuinely can't be a `ctest` assertion and stay manual:
whether a change "looks right" in the 3D view, whether a new GUI panel
renders sensibly, whether the pointing-error lines/mirror-reflection
geometry look correct by eye. Do the launch → wait a few seconds → check
stderr for errors/warnings → kill pattern for those, same as this project
has all along — it's a real verification step, just not one `ctest` can
run unattended.

## Zero-allocation check

FSW code (`src/fsw/ADCS.h/.cpp`, `src/fsw/FDIR.h/.cpp`) must never dynamically
allocate or hold an RNG — see `docs/ALGORITHMS.md`'s hardware-abstraction
note and `FlightTypes.h`'s own comment on why (fixed-size arrays, no heap
allocation after init, matching real embedded/flight coding standards).
Confirm after any change to those files:

```bash
grep -n "std::vector\|push_back\|resize\|<random>" src/fsw/ADCS.h src/fsw/ADCS.cpp src/fsw/FDIR.h src/fsw/FDIR.cpp
```

Expect no output (grep exit code 1). This isn't automated into `ctest`
(a grep isn't really a "test" in the same sense) but should run every time
those files change, the same way a build and the actual test suite do.
