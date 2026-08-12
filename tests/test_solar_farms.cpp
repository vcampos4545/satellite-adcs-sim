// Regression tests for SolarFarms.h/.cpp -- the solar-farm database plus
// its ECI-position helper (data + globe visualization only; see
// SolarFarms.h's own header comment on why there's no auto-targeting
// logic to test here, unlike GroundStations.h's selection/pass-prediction
// machinery -- see test_ground_stations.cpp for that pattern).
#include "test_common.h"
#include "SolarFarms.h"
#include <rigidbody/orbit/OrbitFrames.h>
#include <cmath>

int main()
{
  // solarFarmPositionEci at thetaGstRad=0 must exactly match
  // OrbitFrames::geodeticToECEF (ECEF and ECI coincide when the
  // Earth-rotation angle is zero -- same check test_ground_stations.cpp
  // uses for groundStationPositionEci).
  {
    SolarFarm farm{"Test Farm", 24.4560, 32.7390}; // Benban
    glm::dvec3 expectedEcef = OrbitFrames::geodeticToECEF(farm.latDeg, farm.lonDeg);
    glm::dvec3 eci = solarFarmPositionEci(farm, 0.0);
    double errM = glm::length(eci - expectedEcef);
    CHECK(errM < 1.0, "solarFarmPositionEci(theta=0): matches geodeticToECEF directly (err = %.6f m)", errM);
  }

  // A farm's ECI position must sweep with Earth's rotation -- at
  // thetaGstRad = pi/2, its distance from the rotation axis (Earth's Z)
  // is unchanged (a rotation about Z), but its (x,y) direction differs
  // from the theta=0 case.
  {
    SolarFarm farm{"Test Farm", 0.0, 0.0}; // on the equator, prime meridian
    glm::dvec3 eciAtZero = solarFarmPositionEci(farm, 0.0);
    glm::dvec3 eciAtHalfPi = solarFarmPositionEci(farm, M_PI / 2.0);
    double radiusErr = std::abs(glm::length(eciAtZero) - glm::length(eciAtHalfPi));
    CHECK(radiusErr < 1.0, "solarFarmPositionEci: rotation preserves distance from Earth's center (err = %.6f m)", radiusErr);
    double dotProduct = glm::dot(glm::normalize(eciAtZero), glm::normalize(eciAtHalfPi));
    CHECK(std::abs(dotProduct) < 0.01, "solarFarmPositionEci: a 90deg GMST rotation visibly moves the farm's ECI position (dot = %.6f, want ~0)", dotProduct);
  }

  // SOLAR_FARMS itself: every entry has a real name and a lat/lon within
  // valid geodetic bounds (a real-data sanity check, not a math check --
  // catches a transcription typo like a swapped lat/lon or an out-of-range
  // value).
  {
    bool allValid = true;
    for (const SolarFarm &farm : SOLAR_FARMS)
    {
      if (farm.name == nullptr || farm.name[0] == '\0')
        allValid = false;
      if (farm.latDeg < -90.0 || farm.latDeg > 90.0)
        allValid = false;
      if (farm.lonDeg < -180.0 || farm.lonDeg > 180.0)
        allValid = false;
    }
    CHECK(allValid, "SOLAR_FARMS: every entry has a name and geodetically-valid lat/lon");
    CHECK(SOLAR_FARMS.size() == 50, "SOLAR_FARMS: has the expected 50 entries (got %zu)", SOLAR_FARMS.size());
  }

  TEST_MAIN_END();
}
