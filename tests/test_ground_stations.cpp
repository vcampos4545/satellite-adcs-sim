// Regression tests for GroundStations.h/.cpp -- the ground-station
// database plus the "closest station meeting min elevation" selection
// logic that now drives adcs.target automatically (see
// satellite_adcs_sim.cpp's ground-station reselection comment).
#include "test_common.h"
#include "GroundStations.h"
#include <rigidbody/orbit/OrbitFrames.h>
#include <rigidbody/orbit/OrbitForceModel.h>
#include <rigidbody/orbit/OrbitalElements.h>
#include <rigidbody/orbit/OrbitTime.h>
#include <cmath>
#include <string>

int main()
{
  // groundStationPositionEci at thetaGstRad=0 must exactly match
  // OrbitFrames::geodeticToECEF (ECEF and ECI coincide when the
  // Earth-rotation angle is zero -- same check orbit_validation.cpp
  // uses on the engine side for eciToGeodeticDeg).
  {
    GroundStation station{"Test Station", 39.7392, -104.9903}; // Denver
    glm::dvec3 expectedEcef = OrbitFrames::geodeticToECEF(station.latDeg, station.lonDeg);
    glm::dvec3 eci = groundStationPositionEci(station, 0.0);
    double errM = glm::length(eci - expectedEcef);
    CHECK(errM < 1.0, "groundStationPositionEci(theta=0): matches geodeticToECEF directly (err = %.6f m)", errM);
  }

  // A station's ECI position must sweep with Earth's rotation -- at
  // thetaGstRad = pi/2, its distance from the rotation axis (Earth's Z)
  // should be unchanged (a rotation about Z), but its (x,y) direction
  // should differ from the theta=0 case.
  {
    GroundStation station{"Test Station", 0.0, 0.0}; // on the equator, prime meridian
    glm::dvec3 eciAtZero = groundStationPositionEci(station, 0.0);
    glm::dvec3 eciAtHalfPi = groundStationPositionEci(station, M_PI / 2.0);
    double radiusErr = std::abs(glm::length(eciAtZero) - glm::length(eciAtHalfPi));
    CHECK(radiusErr < 1.0, "groundStationPositionEci: rotation preserves distance from Earth's center (err = %.6f m)", radiusErr);
    double dotProduct = glm::dot(glm::normalize(eciAtZero), glm::normalize(eciAtHalfPi));
    CHECK(std::abs(dotProduct) < 0.01, "groundStationPositionEci: a 90deg GMST rotation visibly moves the station's ECI position (dot = %.6f, want ~0)", dotProduct);
  }

  // A satellite directly overhead a known station (same lat/lon, LEO
  // altitude, at thetaGstRad=0 so ECI/ECEF coincide) must select that
  // station, at ~90deg elevation and slant range ~= altitude.
  {
    const GroundStation &denver = GROUND_STATIONS[2]; // see GROUND_STATIONS' own ordering
    CHECK(std::string(denver.name) == "Denver, CO", "test assumption: GROUND_STATIONS[2] is Denver (was %s)", denver.name);

    double altitudeM = 500e3;
    glm::dvec3 stationEcef = OrbitFrames::geodeticToECEF(denver.latDeg, denver.lonDeg);
    glm::dvec3 satEci = glm::normalize(stationEcef) * (OrbitFrames::EARTH_RADIUS_M + altitudeM);

    glm::dvec3 targetEci;
    const GroundStation *chosen = selectClosestGroundStation(satEci, 0.0, OrbitFrames::DEG2RAD * 10.0, targetEci);
    CHECK(chosen != nullptr && std::string(chosen->name) == "Denver, CO",
          "selectClosestGroundStation: satellite directly overhead Denver selects Denver (got %s)",
          chosen ? chosen->name : "nullptr");

    double rangeM = glm::length(satEci - targetEci);
    double rangeErr = std::abs(rangeM - altitudeM);
    CHECK(rangeErr < 1.0, "selectClosestGroundStation: slant range directly overhead matches altitude (err = %.6f m)", rangeErr);

    double elevationDeg = OrbitFrames::elevationAngleRad(targetEci, satEci) / OrbitFrames::DEG2RAD;
    CHECK(elevationDeg > 89.9, "selectClosestGroundStation: directly-overhead elevation is ~90deg (got %.4f deg)", elevationDeg);
  }

  // With no station meeting an unreasonably strict elevation requirement,
  // selection must return nullptr -- a station outside the footprint is
  // not a viable target, so there's deliberately no "closest regardless
  // of visibility" fallback (the caller, main(), falls back to pointing
  // at the Sun instead).
  {
    glm::dvec3 satEci(OrbitFrames::EARTH_RADIUS_M + 500e3, 0.0, 0.0);
    glm::dvec3 targetEci;
    const GroundStation *chosen = selectClosestGroundStation(satEci, 0.0, OrbitFrames::DEG2RAD * 89.999, targetEci);
    CHECK(chosen == nullptr,
          "selectClosestGroundStation: returns nullptr when no station meets an unreachable elevation requirement (got %s)",
          chosen ? chosen->name : "nullptr");
  }

  // predictGroundStationPasses: structural invariants over the real
  // default-orbit-shaped scenario (24h lookahead, a real minimum
  // elevation) across the whole network.
  {
    OrbitState state = OrbitalElements::circular(500e3, 51.6 * OrbitFrames::DEG2RAD, OrbitFrames::EARTH_RADIUS_M)
                            .toState(TwoBodyGravity{}.mu);
    double epochJd = OrbitTime::julianDate(2026, 3, 20, 12, 0, 0.0);
    double minElevationRad = 10.0 * OrbitFrames::DEG2RAD;

    std::vector<GroundStationPass> passes =
        predictGroundStationPasses(state, epochJd, minElevationRad, 24.0 * 3600.0, 15.0);

    CHECK(!passes.empty(),
          "predictGroundStationPasses: at least one pass predicted over 24h for the default ISS-like orbit across GROUND_STATIONS (got %zu)",
          passes.size());

    bool sorted = true;
    bool allValid = true;
    double prevAosJd = -1.0;
    for (const GroundStationPass &p : passes)
    {
      if (p.aosJd < prevAosJd)
        sorted = false;
      prevAosJd = p.aosJd;
      if (p.losJd <= p.aosJd)
        allValid = false;
      // 0.5deg tolerance for the +/-15s step discretization (AOS/LOS can
      // land up to one step early/late, so the recorded max elevation can
      // undershoot the true peak by a small amount near a shallow pass).
      if (p.maxElevationDeg < 10.0 - 0.5)
        allValid = false;
    }
    CHECK(sorted, "predictGroundStationPasses: passes are sorted by AOS");
    CHECK(allValid,
          "predictGroundStationPasses: every pass has LOS after AOS and max elevation >= the requested minimum (within step tolerance)");
  }

  // Directly-overhead determinism check, mirroring selectClosestGroundStation's
  // own test above: a satellite starting exactly at a known station's
  // zenith must predict a near-90deg pass over that station almost
  // immediately (fine 1s step, short lookahead, so the satellite barely
  // moves from zenith before the check).
  {
    const GroundStation &denver = GROUND_STATIONS[2];
    double altitudeM = 500e3;
    double radiusM = OrbitFrames::EARTH_RADIUS_M + altitudeM;
    double epochJd = OrbitTime::julianDate(2026, 3, 20, 12, 0, 0.0);
    // Denver's actual ECI position *at this epoch* (not its raw ECEF
    // value directly -- gmstRad(epochJd) is generally nonzero, so those
    // differ; predictGroundStationPasses itself computes each station's
    // position this same way every step).
    glm::dvec3 stationEciAtEpoch = groundStationPositionEci(denver, OrbitFrames::gmstRad(epochJd));
    glm::dvec3 satPosEci = glm::normalize(stationEciAtEpoch) * radiusM;

    double circularSpeed = std::sqrt(TwoBodyGravity{}.mu / radiusM);
    glm::dvec3 tangent = glm::normalize(glm::cross(glm::dvec3(0.0, 0.0, 1.0), satPosEci));

    OrbitState state;
    state.position = satPosEci;
    state.velocity = tangent * circularSpeed;
    state.missionTimeS = 0.0;

    std::vector<GroundStationPass> passes =
        predictGroundStationPasses(state, epochJd, 10.0 * OrbitFrames::DEG2RAD, 120.0, 1.0);

    bool foundHighElevationDenverPass = false;
    for (const GroundStationPass &p : passes)
      if (std::string(p.station->name) == "Denver, CO" && p.maxElevationDeg > 89.0)
        foundHighElevationDenverPass = true;
    CHECK(foundHighElevationDenverPass,
          "predictGroundStationPasses: a satellite starting directly overhead Denver predicts a near-90deg Denver pass");
  }

  TEST_MAIN_END();
}
