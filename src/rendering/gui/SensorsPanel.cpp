#include "SensorsPanel.h"
#include <imgui.h>

void drawSensorsTab(ADCS &adcs, SensorTelemetry &telemetry)
{
  ImGui::SeparatorText("IMU");
  glm::vec3 gyroDeg = glm::degrees(adcs.lastGyroBody);
  ImGui::Text("Gyro (deg/s):  %+7.2f  %+7.2f  %+7.2f", gyroDeg.x, gyroDeg.y, gyroDeg.z);
  plotChannel("Gyro rate", telemetry.gyroMagDegS, "deg/s");
  ImGui::Text("Accel (m/s^2): %+7.3f  %+7.3f  %+7.3f", adcs.lastAccelBody.x, adcs.lastAccelBody.y, adcs.lastAccelBody.z);
  plotChannel("Accel", telemetry.accelMagMs2, "m/s^2");

  ImGui::SeparatorText("Magnetometer");
  glm::vec3 fieldUt = adcs.magFieldBody * 1e6f;
  ImGui::Text("Field (uT): %+7.2f  %+7.2f  %+7.2f", fieldUt.x, fieldUt.y, fieldUt.z);
  plotChannel("Field magnitude", telemetry.magFieldMagUt, "uT");
  ImGui::Text("dB/dt: %.2f uT/s", glm::length(adcs.magFieldRateBody) * 1e6f);

  ImGui::SeparatorText("Star Tracker");
  if (adcs.starTrackerValid)
    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.4f, 1.0f), "[VALID] -- primary attitude correction");
  else if (adcs.triadFallbackUsed)
    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "[BLINDED/SLEWING] -- falling back to sun+magnetometer TRIAD");
  else
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "[NO CORRECTION] -- coasting on gyro propagation only");
  ImGui::TextDisabled("Blinded when the boresight comes within the sun-exclusion angle, or while slewing too fast to centroid stars.");
}
