#pragma once
#include <glm/glm.hpp>
#include <rigidbody/orbit/OrbitState.h>
#include <array>
#include <vector>

// A ground station's real-world site: name plus geodetic lat/lon (degrees)
// -- the same spherical-Earth convention OrbitFrames uses throughout this
// project (see its own header comment on why: adequate for this project's
// pointing/visualization accuracy, not a WGS84 ellipsoidal model).
// Altitude is assumed sea level (OrbitFrames::geodeticToECEF's default
// altM=0) -- close enough at LEO ranges for pointing/visibility purposes.
struct GroundStation
{
  const char *name;
  double latDeg;
  double lonDeg;
};

// A handful of major US cities, standing in for real ground station sites
// until this project has an actual network-planning concept (see
// README.md's Scope section) -- picked for geographic spread across the
// continental US so different orbital ground tracks pass over different
// subsets of them, not for any real ground-segment significance. A fixed
// compiled list rather than a loaded data file: six entries has no real
// need for a parser, and this keeps the data co-located with the small
// amount of code that consumes it (see selectClosestGroundStation below).
inline constexpr std::array<GroundStation, 6> GROUND_STATIONS = {{
    {"Seattle, WA", 47.6062, -122.3321},
    {"Los Angeles, CA", 34.0522, -118.2437},
    {"Denver, CO", 39.7392, -104.9903},
    {"Houston, TX", 29.7604, -95.3698},
    {"Chicago, IL", 41.8781, -87.6298},
    {"New York, NY", 40.7128, -74.0060},
}};

// A station's current real ECI position, accounting for Earth's rotation
// -- its ECEF position (fixed to the rotating Earth) is constant, but the
// ECI position that matters for pointing/elevation math against the
// satellite's own (ECI) position sweeps around as Earth turns underneath
// it, via the standard ECEF->ECI rotation by the current Greenwich
// sidereal angle (see OrbitFrames::gmstRad).
glm::dvec3 groundStationPositionEci(const GroundStation &station, double thetaGstRad);

// Returns the closest (by straight-line slant range to the satellite)
// ground station currently within the satellite's footprint (elevation
// >= minElevationRad -- the same criterion drawGroundFootprint's coverage
// circle and predictGroundStationPasses' AOS/LOS both use), or nullptr if
// none currently are. A station outside the footprint is not a viable
// target at all -- there is deliberately no "closest regardless of
// visibility" fallback here; the caller decides what to do with no
// viable ground station (main() falls back to pointing at the Sun). Also
// writes the selected station's current ECI position
// (`groundStationPositionEci`, above) to outTargetEci when non-null.
const GroundStation *selectClosestGroundStation(const glm::dvec3 &satEci, double thetaGstRad,
                                                 double minElevationRad, glm::dvec3 &outTargetEci);

// A single predicted contact window over one ground station: AOS
// (acquisition of signal, elevation first crosses above the minimum) to
// LOS (loss of signal, elevation drops back below it), plus the peak
// elevation reached in between. `station` points into the static
// GROUND_STATIONS array (stable for the program's lifetime).
//
// Only geometric quantities -- everything a real ground-station pass
// schedule needs that this project can currently compute honestly from
// real orbital mechanics. Link budget (data rate/margin), weather, and
// scheduling/priority are deliberately not modeled yet (see
// GroundStationsPanel.h) -- this struct doesn't grow fake fields for them
// ahead of having a real model to compute them from.
struct GroundStationPass
{
  const GroundStation *station;
  double aosJd;
  double losJd;
  double maxElevationDeg;
};

// Predicts every ground-station contact window over the next lookaheadS
// seconds of simulated time, across all of GROUND_STATIONS, sorted by
// AOS. Propagates a *copy* of currentState forward (same force models --
// two-body, J2, drag, SRP, Sun/Moon third-body -- as the real propagator
// and computePredictedOrbitPath, for consistency) in fixed `stepS`
// increments, sampling every station's elevation at each step and
// detecting AOS/LOS as the elevation crosses minElevationRad. AOS/LOS
// timing resolution is therefore +/- stepS (a coarser stepS trades timing
// precision for prediction speed -- see Config::PASS_PREDICTION_STEP_S).
// `missionEpochJd` is the Julian Date at currentState.missionTimeS = 0
// (the same convention computePredictedOrbitPath uses), not the current
// JD.
//
// If a pass is still in progress when the lookahead window ends, it's
// included with losJd set to the window's end instant (a genuinely
// still-open contact at that point, not a fabricated LOS time).
std::vector<GroundStationPass> predictGroundStationPasses(const OrbitState &currentState, double missionEpochJd,
                                                           double minElevationRad, double lookaheadS, double stepS);
