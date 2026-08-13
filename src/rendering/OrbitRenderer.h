#pragma once
#include <vgl/vgl.h>
#include <rigidbody/orbit/OrbitState.h>
#include "core/GroundStations.h"
#include "core/SolarFarms.h"
#include <vector>

// ---------------------------------------------------------------------------
// Global (Earth-scale) orbit visualization: the textured Earth sphere, the
// predicted orbit path, the ground-coverage footprint (3D circle + 2D
// minimap overlay), and the 2D ground-track minimap itself. All operate in
// real ECI meters -- PhysicsWorld's own frame here *is* ECI, so there's no
// separate render scale/offset to track (see main.cpp's
// ORBIT VISUALIZATION comment).
// ---------------------------------------------------------------------------

// Fixes VGL's textured-sphere mesh orientation for *any* equirectangular
// texture (see drawEarth's own .cpp comment for the full derivation): VGL's
// Texture loader flips images vertically on load
// (stbi_set_flip_vertically_on_load), so without this correction, mesh
// local +Y (where the sphere mesh's V=0 texture coordinate lands) samples
// the source image's *bottom* row, not its top -- the sphere renders
// upside-down. drawEarth composes this with an additional meridian
// correction and a time-varying spin (real ECI/GMST alignment, meaningful
// only for a body whose real orientation the scene actually models); this
// constant alone -- no meridian correction, no spin -- is enough to make
// any other textured sphere (Sun, Moon) render right-side-up, since
// neither has a real rotation/prime-meridian model in this project to
// align to in the first place.
inline const glm::quat TEXTURED_SPHERE_POLE_ALIGNMENT =
    glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

// Earth: a textured sphere at the world origin, at its real radius.
//
// Three rotations, composed in order (see drawEarth's own .cpp comment
// for the full derivation of each):
//  1. Meridian correction: a 180 deg rotation about the mesh's own local
//     pole axis, fixing a longitude-only mismatch between VGL's sphere
//     mesh's U=0 seam and the standard equirectangular texture convention
//     (U=0.5 = Greenwich, not U=0) -- without it, ground-station markers
//     (correctly positioned in ECI) rendered ~180 deg away from their
//     real continents.
//  2. Pole alignment: VGL's sphere mesh (MeshGen::sphere) maps its UV v=0
//     (mesh ring index 0) to the mesh's local +Y -- but VGL's Texture
//     loader calls stbi_set_flip_vertically_on_load(true) (see
//     VGL/src/Texture.cpp), so the pixel row that ends up at v=0 is the
//     *bottom* of the source image, not the top. For a standard
//     equirectangular Earth texture (top row = North Pole), that means
//     mesh local +Y actually samples the South Pole, and local -Y
//     samples the North Pole -- the opposite of the naive assumption.
//     A -90 deg rotation about X maps local -Y (the real North Pole
//     content) to world +Z (this scene's "up"/polar convention, the same
//     axis GMST/OrbitFrames rotates about) -- independent of time, it
//     only fixes the mesh's pole axis, it isn't itself Earth's rotation.
//     (A previous version of this used +90 deg, reasoning from the mesh
//     UVs alone without accounting for the texture-loader flip -- that
//     put the correct polar axis in place but with North and South
//     swapped, which is exactly the "sideways, then upside-down" order
//     these two bugs actually showed up in.)
//  3. Spin: +Z is now the correct polar axis, so rotating about it by the
//     current Greenwich sidereal angle is the real ECEF->ECI rotation
//     (same convention orbitState/OrbitFrames.h use) -- this is what
//     actually makes Earth visibly rotate over time.
// Composed as spin * poleAlignment * meridianCorrection (apply the
// meridian correction first, then pole alignment, then spin about the
// now-correct axis -- quaternion composition applies the right-hand
// operand first).
void drawEarth(GUI &gui, const Texture &earthTexture, double currentJd);

// Predicted orbit path: propagates a *copy* of the current orbital state
// forward for one estimated period (vis-viva, from the current
// position/velocity -- exact for the unperturbed two-body case, a good
// approximation with the perturbations included since none of them
// secularly change semi-major axis to first order) using the same force
// models the real propagator uses (two-body, J2, drag, SRP, Sun/Moon
// third-body), sampling numPoints evenly-spaced points. Returns real
// ECI-meter points (world origin = Earth's center, matching
// PhysicsWorld's own frame here), ready to draw directly. Recomputed
// periodically (not every frame) by the caller -- see
// VisualizationConfig::ORBIT_PATH_REFRESH_S -- since nothing here is cheap enough to
// be worth redoing 60 times a second for a path that only drifts slowly
// (mainly from J2) cycle to cycle.
std::vector<glm::vec3> computePredictedOrbitPath(const OrbitState &current, int numPoints, double epochJd);

void drawOrbitPath(GUI &gui, const std::vector<glm::vec3> &pathPoints);

// Coverage half-angle (radians): the angular radius, from Earth's center,
// of the surface circle a satellite at satPosEci can see at or above
// minElevationRad -- shared by the 3D footprint (drawGroundFootprint) and
// the 2D ground-track minimap's footprint overlay, so the two can never
// drift out of sync with each other.
//
// With eta the satellite's nadir angle at which the local horizon
// (elevation = minElevationRad) is reached,
//   sin(eta) = R * cos(eps) / r         (R = Earth radius, r = sat radius, eps = min elevation)
//   rho = pi/2 - eps - eta
// standard ground-coverage geometry (a right triangle formed by Earth's
// center, the satellite, and the point on the horizon circle). Returns
// <= 0 if the satellite is too low/close for any point to reach
// minElevationRad, in which case there's no footprint to draw.
double computeFootprintHalfAngleRad(const glm::dvec3 &satPosEci, double minElevationRad);

// Ground footprint: the small circle on Earth's surface a satellite at
// satPosEci can see at or above minElevationRad, plus a dashed nadir line
// from the surface up to the satellite -- ported from constellation-sim's
// SatelliteRenderer::drawCoverageFootprint (same geometry, this project's
// own real ECI position/PhysicsWorld-frame convention instead of a
// separately-scaled scene).
void drawGroundFootprint(GUI &gui, const glm::dvec3 &satPosEci, double minElevationRad);

// 2D ground-track minimap: the Earth texture as a flat equirectangular
// background image, the ground-track history as a fading trail, the
// footprint circle projected onto the same map, and a marker at the
// current sub-satellite point -- ported from constellation-sim's
// SatelliteRenderer's "GROUND TRACK" panel (drawMercatorWindow's logic,
// in their code, ended up inlined into the panel that calls it; same
// here).
//
// UV flip note: VGL's Texture loader flips images vertically on load
// (stbi_set_flip_vertically_on_load -- see drawEarth's comment on why
// that mattered for the 3D globe's pole alignment). ImGui::Image's UV0/UV1
// arguments compensate for the same flip here, the same way
// constellation-sim's own minimap does -- (0,1)/(1,0) instead of the
// usual (0,0)/(1,1), so the displayed 2D map still reads north-up.
void drawGroundTrackMinimap(const Texture &earthTexture,
                            const std::vector<glm::vec2> &groundTrack,
                            const glm::dvec3 &satPosEci, double thetaGstRad,
                            double minElevationRad);

// Ground station markers: a small sphere at each GROUND_STATIONS entry's
// real, currently-rotated ECI position (groundStationPositionEci --
// recomputed every call from the live Greenwich sidereal angle, so the
// markers visibly sweep with Earth's rotation rather than sitting fixed
// in the inertial frame). Drawn in a neutral color distinct from the
// green target/line pair (drawn separately by the caller for whichever
// station is currently selected as adcs.target) -- the selected one reads
// as "the green-highlighted station among the white ones," not a
// separately-styled marker of its own.
void drawGroundStations(GUI &gui, double thetaGstRad);

// Solar farm markers: same shape as drawGroundStations, one sphere per
// SOLAR_FARMS entry at its real, currently-rotated ECI position
// (solarFarmPositionEci) -- a gold tone distinct from ground stations'
// neutral gray so both marker sets read separately on the globe. Reflect-
// Orbital mission candidate targets; not wired into REFLECT's auto-
// targeting (see SolarFarms.h's own header comment).
void drawSolarFarms(GUI &gui, double thetaGstRad);
