#include "SimulationPanel.h"
#include "core/Config.h"
#include <imgui.h>
#include <random>
#include <algorithm>
#include <cmath>

void drawSimulationTab(SimControls &sim, EpochControls &epoch,
                       std::vector<ReactionWheel *> &wheels, RigidBody *body, ADCS &adcs)
{
  ImGui::SeparatorText("Time Controls");
  ImGui::Checkbox("Pause", &sim.paused);
  ImGui::TextDisabled("Physics and FSW freeze; camera/UI stay live.");

  ImGui::BeginDisabled(!sim.paused);
  if (ImGui::Button("Step"))
    sim.stepRequested = true;
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::TextDisabled("(advances one %.2fs simulated step, then re-pauses; only while paused)", Config::TIME_STEP_S);

  ImGui::DragFloat("Speed", &sim.timeScale, 0.5f, 0.1f, 100.0f, "%.1fx", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp);
  ImGui::TextDisabled("Scales simulated time per real second -- applies uniformly to orbit,");
  ImGui::TextDisabled("physics, and the FSW cycle together, so control behavior stays correct at any speed.");

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
    adcs.requestDesaturation();
  ImGui::SameLine();
  ImGui::TextDisabled("(runs in the background until wheel momentum is low; keeps pointing)");

  ImGui::Checkbox("Auto-desaturate when a wheel gets close to saturated", &adcs.desatAutoTriggerEnabled);
  ImGui::DragFloat("Auto-trigger threshold (fraction)", &adcs.desatTriggerSaturation, 0.01f, 0.5f, 1.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::DragFloat("Auto-stop threshold (fraction)", &adcs.desatStopSaturation, 0.01f, 0.0f, 0.9f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::DragFloat("Desaturation gain", &adcs.desatGain, 0.001f, 0.0f, 1.0f, "%.4f");
}
