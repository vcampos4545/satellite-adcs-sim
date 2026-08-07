#include "OrbitRenderer.h"
#include "core/Config.h"
#include <rigidbody/orbit/OrbitalElements.h>
#include <rigidbody/orbit/OrbitForceModel.h>
#include <rigidbody/orbit/OrbitPropagator.h>
#include <rigidbody/orbit/OrbitFrames.h>
#include <rigidbody/orbit/ThirdBodyGravity.h>
#include <rigidbody/orbit/AtmosphericDrag.h>
#include <rigidbody/orbit/SolarRadiationPressure.h>
#include <imgui.h>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <cstdio>
#include <memory>

void drawEarth(GUI &gui, const Texture &earthTexture, double currentJd)
{
  glm::quat poleAlignment = glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
  float gmst = static_cast<float>(OrbitFrames::gmstRad(currentJd));
  glm::quat spin = glm::angleAxis(gmst, glm::vec3(0.0f, 0.0f, 1.0f));
  glm::quat earthRotation = spin * poleAlignment;
  gui.drawTexturedSphere(glm::vec3(0.0f), static_cast<float>(OrbitFrames::EARTH_RADIUS_M), earthRotation, earthTexture);
}

std::vector<glm::vec3> computePredictedOrbitPath(const OrbitState &current, int numPoints, double epochJd)
{
  double mu = TwoBodyGravity{}.mu;
  double rMag = glm::length(current.position);
  double vMagSq = glm::dot(current.velocity, current.velocity);
  double semiMajorAxisM = 1.0 / (2.0 / rMag - vMagSq / mu); // vis-viva
  double period = 2.0 * M_PI * std::sqrt(semiMajorAxisM * semiMajorAxisM * semiMajorAxisM / mu);

  OrbitPropagator pathPropagator;
  pathPropagator.addForceModel(std::make_unique<TwoBodyGravity>());
  pathPropagator.addForceModel(std::make_unique<J2Perturbation>());
  pathPropagator.addForceModel(
      std::make_unique<AtmosphericDrag>(Config::SPACECRAFT_CROSS_SECTION_M2, Config::SPACECRAFT_MASS_KG));
  auto sunGravity = std::make_unique<ThirdBodyGravity>(ThirdBodyType::Sun);
  auto moonGravity = std::make_unique<ThirdBodyGravity>(ThirdBodyType::Moon);
  auto srp = std::make_unique<SolarRadiationPressure>(Config::SPACECRAFT_CROSS_SECTION_M2, Config::SPACECRAFT_MASS_KG);
  sunGravity->epochJd = epochJd;
  moonGravity->epochJd = epochJd;
  srp->epochJd = epochJd;
  pathPropagator.addForceModel(std::move(sunGravity));
  pathPropagator.addForceModel(std::move(moonGravity));
  pathPropagator.addForceModel(std::move(srp));

  OrbitState state = current;
  double dt = period / numPoints;

  std::vector<glm::vec3> points;
  points.reserve(numPoints + 1);
  points.push_back(glm::vec3(state.position));
  for (int i = 0; i < numPoints; i++)
  {
    pathPropagator.step(state, dt);
    points.push_back(glm::vec3(state.position));
  }
  return points;
}

void drawOrbitPath(GUI &gui, const std::vector<glm::vec3> &pathPoints)
{
  const glm::vec3 pathColor{0.3f, 0.7f, 1.0f};
  for (size_t i = 0; i + 1 < pathPoints.size(); i++)
    gui.drawLine(pathPoints[i], pathPoints[i + 1], pathColor, 1.5f);
}

double computeFootprintHalfAngleRad(const glm::dvec3 &satPosEci, double minElevationRad)
{
  double r = glm::length(satPosEci);
  double R = OrbitFrames::EARTH_RADIUS_M;
  double eta = std::asin(glm::clamp(R * std::cos(minElevationRad) / r, 0.0, 1.0));
  return glm::half_pi<double>() - minElevationRad - eta;
}

void drawGroundFootprint(GUI &gui, const glm::dvec3 &satPosEci, double minElevationRad)
{
  double rho = computeFootprintHalfAngleRad(satPosEci, minElevationRad);
  if (rho <= 0.0)
    return; // satellite too low/close for any point to reach minElevationRad

  float cosRho = static_cast<float>(std::cos(rho));
  float sinRho = static_cast<float>(std::sin(rho));

  glm::vec3 sub = glm::normalize(glm::vec3(satPosEci)); // sub-satellite direction

  // Orthonormal basis perpendicular to sub, for tracing the small circle.
  glm::vec3 ref = (std::abs(glm::dot(sub, glm::vec3(0, 0, 1))) > 0.99f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 0, 1);
  glm::vec3 t1 = glm::normalize(glm::cross(sub, ref));
  glm::vec3 t2 = glm::cross(sub, t1); // already unit (sub, t1 orthonormal)

  const int segments = 96;
  const float surfaceOffset = static_cast<float>(OrbitFrames::EARTH_RADIUS_M) * 1.001f; // just above the surface, avoids z-fighting
  const glm::vec3 footprintColor{0.0f, 0.85f, 1.0f};

  glm::vec3 prev{0.0f};
  for (int i = 0; i <= segments; i++)
  {
    float angle = glm::two_pi<float>() * i / segments;
    glm::vec3 dir = sub * cosRho + (t1 * std::cos(angle) + t2 * std::sin(angle)) * sinRho;
    glm::vec3 pt = dir * surfaceOffset;
    if (i > 0)
      gui.drawLine(prev, pt, footprintColor, 1.8f);
    prev = pt;
  }

  // Dashed nadir line, surface straight up to the satellite.
  glm::vec3 surfacePt = sub * surfaceOffset;
  glm::vec3 satPt = glm::vec3(satPosEci);
  const int dashSegments = 20;
  for (int i = 0; i < dashSegments; i += 2) // skip every other -- dashed look
  {
    glm::vec3 a = glm::mix(surfacePt, satPt, static_cast<float>(i) / dashSegments);
    glm::vec3 b = glm::mix(surfacePt, satPt, static_cast<float>(i + 1) / dashSegments);
    gui.drawLine(a, b, footprintColor * 0.8f, 0.8f);
  }
}

void drawGroundTrackMinimap(const Texture &earthTexture,
                            const std::vector<glm::vec2> &groundTrack,
                            const glm::dvec3 &satPosEci, double thetaGstRad,
                            double minElevationRad)
{
  if (!ImGui::BeginChild("##ground_track", ImVec2(0.0f, 220.0f), false, ImGuiWindowFlags_NoScrollbar))
  {
    ImGui::EndChild();
    return;
  }

  ImVec2 cp = ImGui::GetCursorScreenPos();
  ImVec2 cs = ImGui::GetContentRegionAvail();

  ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(earthTexture.id())),
               cs, ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));

  ImDrawList *dl = ImGui::GetWindowDrawList();

  // Equirectangular projection: lon in [-180,180] -> x in [0, width],
  // lat in [-90,90] -> y in [0, height] (north at the top).
  auto toScreen = [&](float latDeg, float lonDeg) -> ImVec2
  {
    return ImVec2(cp.x + (lonDeg + 180.0f) / 360.0f * cs.x,
                  cp.y + (90.0f - latDeg) / 180.0f * cs.y);
  };

  // Ground-track trail, fading from dim to bright toward the current
  // position. Segments that cross the +-180 deg seam (a huge apparent
  // longitude jump) are skipped rather than drawn as a spurious line all
  // the way across the map.
  int trailCount = static_cast<int>(groundTrack.size());
  for (int i = 1; i < trailCount; i++)
  {
    float lon0 = groundTrack[i - 1].y, lon1 = groundTrack[i].y;
    if (std::abs(lon1 - lon0) > 90.0f)
      continue;
    float age = static_cast<float>(i) / trailCount;
    ImU32 col = IM_COL32(static_cast<int>(20 + 20 * age), static_cast<int>(120 + 135 * age), static_cast<int>(220 - 20 * age), 220);
    dl->AddLine(toScreen(groundTrack[i - 1].x, groundTrack[i - 1].y),
                toScreen(groundTrack[i].x, groundTrack[i].y), col, 1.5f);
  }

  // Footprint circle, projected onto the map -- traced the same way the 3D
  // version does (see drawGroundFootprint), each point converted to
  // geodetic individually since the small-circle-around-nadir shape isn't
  // preserved under an equirectangular projection (especially near the
  // poles).
  double rho = computeFootprintHalfAngleRad(satPosEci, minElevationRad);
  if (rho > 0.0)
  {
    glm::dvec3 sub = glm::normalize(satPosEci);
    glm::dvec3 ref = (std::abs(sub.z) > 0.99) ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 0.0, 1.0);
    glm::dvec3 t1 = glm::normalize(glm::cross(sub, ref));
    glm::dvec3 t2 = glm::cross(sub, t1);
    double cosRho = std::cos(rho), sinRho = std::sin(rho);

    const int segments = 72;
    bool hasPrev = false;
    ImVec2 prevPt{};
    float prevLon = 0.0f;
    for (int i = 0; i <= segments; i++)
    {
      double angle = glm::two_pi<double>() * i / segments;
      glm::dvec3 dir = sub * cosRho + (t1 * std::cos(angle) + t2 * std::sin(angle)) * sinRho;
      OrbitFrames::Geodetic geo = OrbitFrames::eciToGeodeticDeg(dir * OrbitFrames::EARTH_RADIUS_M, thetaGstRad);
      ImVec2 pt = toScreen(static_cast<float>(geo.latDeg), static_cast<float>(geo.lonDeg));
      if (hasPrev && std::abs(static_cast<float>(geo.lonDeg) - prevLon) < 90.0f)
        dl->AddLine(prevPt, pt, IM_COL32(0, 210, 255, 160), 1.2f);
      prevPt = pt;
      prevLon = static_cast<float>(geo.lonDeg);
      hasPrev = true;
    }
  }

  OrbitFrames::Geodetic nowGeo = OrbitFrames::eciToGeodeticDeg(satPosEci, thetaGstRad);
  ImVec2 satPx = toScreen(static_cast<float>(nowGeo.latDeg), static_cast<float>(nowGeo.lonDeg));
  dl->AddCircleFilled(satPx, 5.5f, IM_COL32(0, 220, 255, 255));
  dl->AddCircle(satPx, 7.5f, IM_COL32(255, 255, 255, 200), 16, 1.2f);

  char coordBuf[64];
  std::snprintf(coordBuf, sizeof(coordBuf), "%.2f %s  %.2f %s",
                std::abs(nowGeo.latDeg), nowGeo.latDeg >= 0 ? "N" : "S",
                std::abs(nowGeo.lonDeg), nowGeo.lonDeg >= 0 ? "E" : "W");
  dl->AddText(ImVec2(cp.x + 4.0f, cp.y + cs.y - 16.0f), IM_COL32(220, 220, 220, 220), coordBuf);

  ImGui::EndChild();
}
