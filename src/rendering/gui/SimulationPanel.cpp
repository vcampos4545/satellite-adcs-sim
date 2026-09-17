#include "SimulationPanel.h"
#include <imgui.h>
#include <random>
#include <algorithm>
#include <cmath>

void drawSimulationTab(SimControls &sim, EpochControls &epoch,
                       std::vector<ReactionWheel *> &wheels, RigidBody *body, ADCS &adcs)
{
  ImGui::SeparatorText("Time Controls");
  ImGui::TextDisabled("Scales real elapsed time before it reaches the fixed-rate FSW/physics loop --");
  ImGui::TextDisabled("each step is still exactly SatelliteConfig::TIME_STEP_S; this just changes how many");
  ImGui::TextDisabled("happen per real second. Falls behind smoothly (not a freeze) if a value here");
  ImGui::TextDisabled("would need more steps per frame than SatelliteConfig::FSW_TIMER_MAX_S allows.");
  ImGui::SliderFloat("Time Scale", &sim.timeScale, 0.0f, 500.0f, "%.1fx");
  if (ImGui::Button("Pause"))
    sim.timeScale = 0.0f;
  ImGui::SameLine();
  if (ImGui::Button("1x"))
    sim.timeScale = 1.0f;
  ImGui::SameLine();
  if (ImGui::Button("10x"))
    sim.timeScale = 10.0f;
  ImGui::SameLine();
  if (ImGui::Button("60x"))
    sim.timeScale = 60.0f;
  ImGui::SameLine();
  if (ImGui::Button("200x"))
    sim.timeScale = 200.0f;

  ImGui::Checkbox("VSync", &sim.vsyncEnabled);
  ImGui::TextDisabled("Uncheck for an uncapped render rate -- e.g. to read true render performance");
  ImGui::TextDisabled("off the FPS overlay instead of the display's refresh rate.");

  ImGui::SeparatorText("Mission Epoch (UTC)");
  ImGui::TextDisabled("What real calendar date orbitState's mission clock started at --");
  ImGui::TextDisabled("Sun direction and Earth's rendered rotation both follow this live.");
  int ymd[3] = {epoch.year, epoch.month, epoch.day};
  if (ImGui::InputInt3("Year / Month / Day", ymd))
  {
    epoch.year = ymd[0];
    epoch.month = glm::clamp(ymd[1], 1, 12);
    epoch.day = glm::clamp(ymd[2], 1, 31);
  }
  int hms[2] = {epoch.hour, epoch.minute};
  if (ImGui::InputInt2("Hour / Minute", hms))
  {
    epoch.hour = glm::clamp(hms[0], 0, 23);
    epoch.minute = glm::clamp(hms[1], 0, 59);
  }
  ImGui::DragFloat("Second", &epoch.second, 1.0f, 0.0f, 59.999f, "%.3f", ImGuiSliderFlags_AlwaysClamp);

  ImGui::SeparatorText("Disturbances");
  ImGui::DragFloat("Tumble kick (rad/s)", &sim.tumbleKickRadS, 0.05f, 0.0f, 5.0f, "%.2f");
  if (ImGui::Button("Kick into random tumble [T]"))
  {
    static std::mt19937 tumbleRng(std::random_device{}());
    std::uniform_real_distribution<float> d(-sim.tumbleKickRadS, sim.tumbleKickRadS);
    body->angularVelocity = glm::vec3(d(tumbleRng), d(tumbleRng), d(tumbleRng));
  }
  ImGui::SameLine();
  if (ImGui::Button("Detumble Now"))
    body->angularVelocity = glm::vec3(0.0f);
  ImGui::TextDisabled("Directly zeroes the rigid body's angular velocity -- a debug/reset action, not");
  ImGui::TextDisabled("something real flight software could do; ADCS's own DETUMBLE mode actually damps");
  ImGui::TextDisabled("rate via actuators over time, and also enters/exits it on its own above a rate");
  ImGui::TextDisabled("threshold -- see the FSW tab.");

  ImGui::SeparatorText("Reaction Wheel Faults");
  ImGui::TextDisabled("Manual only -- pick a wheel and a fault, same healthFactor model FDIR reacts to.");
  static float degradeSeverity = 0.3f; // shared by every "Degrade" button below
  ImGui::SliderFloat("Degrade severity (healthFactor)", &degradeSeverity, 0.05f, 0.95f, "%.2f");
  for (size_t i = 0; i < wheels.size(); i++)
  {
    ImGui::PushID(static_cast<int>(i));
    ImGui::Text("Wheel %zu (health %.0f%%)", i, wheels[i]->healthFactor * 100.0f);
    ImGui::SameLine();
    if (ImGui::SmallButton("Fail"))
      wheels[i]->healthFactor = 0.0f;
    ImGui::SameLine();
    if (ImGui::SmallButton("Degrade"))
      wheels[i]->healthFactor = degradeSeverity;
    ImGui::SameLine();
    if (ImGui::SmallButton("Repair"))
      wheels[i]->healthFactor = 1.0f;
    ImGui::PopID();
  }
  if (ImGui::Button("Repair all wheels"))
    for (auto *w : wheels)
      w->healthFactor = 1.0f;

  ImGui::SeparatorText("Momentum Desaturation");
  float maxWheelSat = 0.0f;
  for (auto *w : wheels)
    maxWheelSat = std::max(maxWheelSat, std::abs(w->getSaturationRatio()));
  ImGui::Text("Peak wheel saturation: %.0f%%", maxWheelSat * 100.0f);
  if (adcs.desatActive)
    ImGui::TextColored(ImVec4(0.2f, 0.7f, 1.0f, 1.0f), "Desaturating via magnetorquers...");
  else
    ImGui::TextDisabled("Idle");

  if (ImGui::Button("Desaturate Wheels Now"))
    for (auto *w : wheels)
      w->currentSpeed = 0.0f;
  ImGui::TextDisabled("Directly zeroes each wheel's speed (momentum) -- a debug/reset action, not");
  ImGui::TextDisabled("something real flight software could do; ADCS's own desaturation actually bleeds");
  ImGui::TextDisabled("momentum off via magnetorquers over time -- see the auto-trigger controls below.");

  ImGui::Checkbox("Auto-desaturate when a wheel gets close to saturated", &adcs.desatAutoTriggerEnabled);
  ImGui::DragFloat("Auto-trigger threshold (fraction)", &adcs.desatTriggerSaturation, 0.01f, 0.5f, 1.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::DragFloat("Auto-stop threshold (fraction)", &adcs.desatStopSaturation, 0.01f, 0.0f, 0.9f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::DragFloat("Desaturation gain", &adcs.desatGain, 0.001f, 0.0f, 1.0f, "%.4f");
}
