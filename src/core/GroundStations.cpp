#include "GroundStations.h"
#include "Config.h"
#include <rigidbody/orbit/OrbitFrames.h>
#include <rigidbody/orbit/OrbitTime.h>
#include <rigidbody/orbit/OrbitForceModel.h>
#include <rigidbody/orbit/OrbitPropagator.h>
#include <rigidbody/orbit/ThirdBodyGravity.h>
#include <rigidbody/orbit/AtmosphericDrag.h>
#include <rigidbody/orbit/SolarRadiationPressure.h>
#include <limits>
#include <algorithm>
#include <memory>

glm::dvec3 groundStationPositionEci(const GroundStation &station, double thetaGstRad)
{
  glm::dvec3 ecef = OrbitFrames::geodeticToECEF(station.latDeg, station.lonDeg);
  return OrbitFrames::ecefToECI(ecef, thetaGstRad);
}

const GroundStation *selectClosestGroundStation(const glm::dvec3 &satEci, double thetaGstRad,
                                                 double minElevationRad, glm::dvec3 &outTargetEci)
{
  const GroundStation *best = nullptr;
  double bestRangeM = std::numeric_limits<double>::max();
  glm::dvec3 bestEci{0.0};

  for (const GroundStation &station : GROUND_STATIONS)
  {
    glm::dvec3 stationEci = groundStationPositionEci(station, thetaGstRad);
    double elevationRad = OrbitFrames::elevationAngleRad(stationEci, satEci);
    if (elevationRad < minElevationRad)
      continue; // outside the footprint -- not a viable target at all

    double rangeM = glm::length(satEci - stationEci);
    if (rangeM < bestRangeM)
    {
      bestRangeM = rangeM;
      best = &station;
      bestEci = stationEci;
    }
  }

  if (best)
    outTargetEci = bestEci;
  return best;
}

std::vector<GroundStationPass> predictGroundStationPasses(const OrbitState &currentState, double missionEpochJd,
                                                           double minElevationRad, double lookaheadS, double stepS)
{
  OrbitPropagator propagator;
  propagator.addForceModel(std::make_unique<TwoBodyGravity>());
  propagator.addForceModel(std::make_unique<J2Perturbation>());
  propagator.addForceModel(
      std::make_unique<AtmosphericDrag>(Config::SPACECRAFT_CROSS_SECTION_M2, Config::SPACECRAFT_MASS_KG));
  auto sunGravity = std::make_unique<ThirdBodyGravity>(ThirdBodyType::Sun);
  auto moonGravity = std::make_unique<ThirdBodyGravity>(ThirdBodyType::Moon);
  auto srp = std::make_unique<SolarRadiationPressure>(Config::SPACECRAFT_CROSS_SECTION_M2, Config::SPACECRAFT_MASS_KG);
  sunGravity->epochJd = missionEpochJd;
  moonGravity->epochJd = missionEpochJd;
  srp->epochJd = missionEpochJd;
  propagator.addForceModel(std::move(sunGravity));
  propagator.addForceModel(std::move(moonGravity));
  propagator.addForceModel(std::move(srp));

  OrbitState state = currentState;

  // Per-station in-progress-pass tracking, indexed the same as
  // GROUND_STATIONS.
  struct InProgress
  {
    bool active = false;
    double aosJd = 0.0;
    double maxElevationDeg = -90.0;
  };
  std::array<InProgress, GROUND_STATIONS.size()> inProgress{};

  std::vector<GroundStationPass> passes;
  int numSteps = static_cast<int>(lookaheadS / stepS);

  for (int i = 0; i <= numSteps; i++)
  {
    double jdNow = OrbitTime::advance(missionEpochJd, state.missionTimeS);
    double thetaGst = OrbitFrames::gmstRad(jdNow);

    for (size_t s = 0; s < GROUND_STATIONS.size(); s++)
    {
      glm::dvec3 stationEci = groundStationPositionEci(GROUND_STATIONS[s], thetaGst);
      double elevationRad = OrbitFrames::elevationAngleRad(stationEci, state.position);
      bool visible = elevationRad >= minElevationRad;
      InProgress &ip = inProgress[s];

      if (visible && !ip.active)
      {
        ip.active = true;
        ip.aosJd = jdNow;
        ip.maxElevationDeg = elevationRad / OrbitFrames::DEG2RAD;
      }
      else if (visible && ip.active)
      {
        ip.maxElevationDeg = std::max(ip.maxElevationDeg, elevationRad / OrbitFrames::DEG2RAD);
      }
      else if (!visible && ip.active)
      {
        passes.push_back({&GROUND_STATIONS[s], ip.aosJd, jdNow, ip.maxElevationDeg});
        ip.active = false;
        ip.maxElevationDeg = -90.0;
      }
    }

    if (i < numSteps)
      propagator.step(state, stepS);
  }

  // Any station still visible when the lookahead window ends is a
  // genuinely still-open pass -- record it with LOS at the window's own
  // end instant, not a fabricated later time.
  double jdEnd = OrbitTime::advance(missionEpochJd, state.missionTimeS);
  for (size_t s = 0; s < GROUND_STATIONS.size(); s++)
    if (inProgress[s].active)
      passes.push_back({&GROUND_STATIONS[s], inProgress[s].aosJd, jdEnd, inProgress[s].maxElevationDeg});

  std::sort(passes.begin(), passes.end(),
            [](const GroundStationPass &a, const GroundStationPass &b) { return a.aosJd < b.aosJd; });
  return passes;
}
