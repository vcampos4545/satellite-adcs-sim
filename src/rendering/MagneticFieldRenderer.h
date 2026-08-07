#pragma once
#include <vgl/vgl.h>
#include <rigidbody/environment/central_body/CentralBodyMagneticField.h>
#include <vector>

// ---------------------------------------------------------------------------
// Magnetic field visualization: a single local arrow at the satellite (the
// vector that actually drives magnetorquer/B-dot FSW) plus a globally
// traced dipole field-line loop set (a cosmetic, textbook/magnetosphere-
// style visualization of the same field model, distinct from what FSW
// reads).
// ---------------------------------------------------------------------------

// Visualizes the ambient field the satellite is actually sampling, as a
// single arrow at the body -- what matters for reading FSW behavior
// against the field (e.g. B-dot detumble, magnetorquer dipole direction)
// is this local vector, not the global field geometry (see
// traceDipoleFieldLines/drawMagneticFieldLines below for that).
void drawMagneticField(GUI &gui, const glm::vec3 &fieldWorldT, glm::vec3 satPos);

// One traced dipole field line, plus which magnetic hemisphere it was
// seeded from (for two-tone coloring -- see drawMagneticFieldLines).
struct FieldLine
{
  std::vector<glm::vec3> points;
  bool seededNorth;
};

// Traces closed dipole field-line loops for the global magnetic-field
// visualization (distinct from drawMagneticField's local sample arrow
// above): seeds a colatitude/azimuth grid near each magnetic pole on
// Earth's real surface and integrates outward along the *unit* field
// direction (RK4 in arclength, not time -- this is a streamline, not a
// trajectory) until the line either returns near the surface (loop
// closed) or exceeds a safety radius (near-axis seeds produce very large
// loops that can be slow to close).
//
// Seed colatitude is measured from CentralBodyMagneticField's own
// `rotationAxisWorld` (world +Z by default), not the true ~11-degree-
// tilted dipole axis -- close enough for seed *placement* (the traced
// geometry itself is exact, since it integrates the real sampled field
// at every step) that the loops still visibly fan out from both poles,
// without needing a public accessor for the engine's private dipole-axis
// helper just for this cosmetic seeding choice.
//
// The field is a fixed-inertial tilted dipole (no time dependence in
// this model -- see CentralBodyMagneticField's own header comment), so
// this only needs to run once at startup, not every frame.
std::vector<FieldLine> traceDipoleFieldLines(const CentralBodyMagneticField &magField);

// Draws the traced dipole loops in two tones by seed hemisphere (cyan for
// north, amber for south) -- readable at a glance which pole a given loop
// connects to, similar to how textbook/magnetosphere diagrams color
// field lines by polarity.
void drawMagneticFieldLines(GUI &gui, const std::vector<FieldLine> &lines);
