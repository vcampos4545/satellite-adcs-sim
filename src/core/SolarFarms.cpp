#include "SolarFarms.h"
#include <rigidbody/orbit/OrbitFrames.h>

glm::dvec3 solarFarmPositionEci(const SolarFarm &farm, double thetaGstRad)
{
  glm::dvec3 ecef = OrbitFrames::geodeticToECEF(farm.latDeg, farm.lonDeg);
  return OrbitFrames::ecefToECI(ecef, thetaGstRad);
}
