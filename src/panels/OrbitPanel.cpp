#include "OrbitPanel.h"
#include "core/VisualizationConfig.h"
#include "rendering/OrbitRenderer.h"
#include <rigidbody/orbit/OrbitalElements.h>
#include <rigidbody/orbit/OrbitForceModel.h>
#include <rigidbody/orbit/OrbitFrames.h>
#include <glm/gtc/constants.hpp>
#include <imgui.h>

void drawOrbitTab(const OrbitState &orbitState, const Texture &earthTexture,
                  const std::vector<glm::vec2> &groundTrack, double currentJd)
{
  double mu = TwoBodyGravity{}.mu;
  OrbitalElements elements = OrbitalElements::fromState(orbitState, mu);

  double radiusM = glm::length(orbitState.position);
  double altitudeKm = (radiusM - OrbitFrames::EARTH_RADIUS_M) / 1000.0;
  double apogeeKm = (elements.semiMajorAxisM * (1.0 + elements.eccentricity) - OrbitFrames::EARTH_RADIUS_M) / 1000.0;
  double perigeeKm = (elements.semiMajorAxisM * (1.0 - elements.eccentricity) - OrbitFrames::EARTH_RADIUS_M) / 1000.0;
  double periodMin = elements.periodS(mu) / 60.0;
  double meanMotionRevDay = elements.meanMotionRadS(mu) * 86400.0 / glm::two_pi<double>();

  ImGui::SeparatorText("Orbital State");
  ImGui::TextDisabled("From the real propagated state, not the commanded initial condition --");
  ImGui::TextDisabled("drifts from it as J2 acts on the orbit (see docs/ALGORITHMS.md).");
  ImGui::Text("Altitude: %.2f km", altitudeKm);
  ImGui::Text("Apogee / Perigee altitude: %.2f / %.2f km", apogeeKm, perigeeKm);
  ImGui::Text("Semi-major axis: %.1f km", elements.semiMajorAxisM / 1000.0);
  ImGui::Text("Eccentricity: %.5f", elements.eccentricity);
  ImGui::Text("Inclination: %.3f deg", glm::degrees(elements.inclinationRad));
  ImGui::Text("RAAN: %.3f deg", glm::degrees(elements.raanRad));
  ImGui::Text("Argument of periapsis: %.3f deg", glm::degrees(elements.argPeriapsisRad));
  ImGui::Text("True anomaly: %.3f deg", glm::degrees(elements.trueAnomalyRad));
  ImGui::Text("Period: %.2f min", periodMin);
  ImGui::Text("Mean motion: %.4f rev/day", meanMotionRevDay);

  ImGui::SeparatorText("Ground Track");
  double gmst = OrbitFrames::gmstRad(currentJd);
  drawGroundTrackMinimap(earthTexture, groundTrack, orbitState.position, gmst,
                         glm::radians(static_cast<double>(VisualizationConfig::FOOTPRINT_MIN_ELEVATION_DEG)));
}
