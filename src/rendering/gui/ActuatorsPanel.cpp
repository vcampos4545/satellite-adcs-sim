#include "ActuatorsPanel.h"
#include <imgui.h>
#include <glm/gtc/constants.hpp>
#include <cstdio>

// Status text/color for one actuator, same priority order and color
// convention the 3D visualization uses (drawReactionWheels/
// drawMagnetorquers): a fault (dead/degraded) always outranks saturation,
// since a degraded unit reporting "saturated" would be misleading about
// what's actually wrong with it.
static void wheelStatus(const ReactionWheel *w, const char *&outText, ImVec4 &outColor)
{
  if (w->healthFactor <= 0.01f)
  {
    outText = "FAILED";
    outColor = ImVec4(1.0f, 0.2f, 0.2f, 1.0f);
  }
  else if (w->healthFactor < 0.99f)
  {
    outText = "DEGRADED";
    outColor = ImVec4(0.85f, 0.1f, 0.85f, 1.0f);
  }
  else if (w->isSaturated())
  {
    outText = "SATURATED";
    outColor = ImVec4(1.0f, 0.4f, 0.0f, 1.0f);
  }
  else
  {
    outText = "OK";
    outColor = ImVec4(0.3f, 1.0f, 0.4f, 1.0f);
  }
}

static void torquerStatus(const Magnetorquer *m, const char *&outText, ImVec4 &outColor)
{
  if (m->healthFactor <= 0.01f)
  {
    outText = "FAILED";
    outColor = ImVec4(1.0f, 0.2f, 0.2f, 1.0f);
  }
  else if (m->healthFactor < 0.99f)
  {
    outText = "DEGRADED";
    outColor = ImVec4(0.85f, 0.1f, 0.85f, 1.0f);
  }
  else if (std::abs(m->getSaturationRatio()) >= 0.99f)
  {
    outText = "SATURATED";
    outColor = ImVec4(1.0f, 0.4f, 0.0f, 1.0f);
  }
  else
  {
    outText = "OK";
    outColor = ImVec4(0.3f, 1.0f, 0.4f, 1.0f);
  }
}

void drawActuatorsTab(ADCS &adcs, const std::vector<ReactionWheel *> &wheels,
                      const std::vector<Magnetorquer *> &magnetorquers)
{
  ImGui::SeparatorText("Reaction Wheels");
  for (size_t i = 0; i < wheels.size(); i++)
  {
    const ReactionWheel *w = wheels[i];
    const char *statusText;
    ImVec4 color;
    wheelStatus(w, statusText, color);

    float rpm = w->currentSpeed * 60.0f / (2.0f * glm::pi<float>());
    ImGui::Text("Wheel %zu", i);
    ImGui::SameLine();
    ImGui::TextColored(color, "[%s]", statusText);
    ImGui::Text("  cmd: %+.5f Nm   speed: %+.0f RPM   sat: %.0f%%",
                w->commandedTorque, rpm, std::abs(w->getSaturationRatio()) * 100.0f);
  }

  ImGui::SeparatorText("Magnetorquers");
  for (size_t i = 0; i < magnetorquers.size(); i++)
  {
    const Magnetorquer *m = magnetorquers[i];
    const char *statusText;
    ImVec4 color;
    torquerStatus(m, statusText, color);

    ImGui::Text("Rod %zu", i);
    ImGui::SameLine();
    ImGui::TextColored(color, "[%s]", statusText);
    ImGui::Text("  cmd: %+.3f A*m^2   sat: %.0f%%",
                m->commandedDipoleMoment, std::abs(m->getSaturationRatio()) * 100.0f);
  }

  ImGui::SeparatorText("Manual Control");
  ImGui::Checkbox("Enable manual override", &adcs.manualOverride);
  if (adcs.manualOverride)
  {
    ImGui::TextDisabled("Directly commands hardware; FSW guidance/control/allocation is skipped.");

    for (size_t i = 0; i < wheels.size() && i < (size_t)NUM_WHEELS; i++)
    {
      char label[32];
      std::snprintf(label, sizeof(label), "Wheel %zu (Nm)", i);
      ImGui::SliderFloat(label, &adcs.manualWheelTorqueNm[i], -wheels[i]->maxTorque, wheels[i]->maxTorque, "%.5f");
    }

    for (size_t i = 0; i < magnetorquers.size() && i < (size_t)NUM_TORQUERS; i++)
    {
      char label[32];
      std::snprintf(label, sizeof(label), "Rod %zu (A*m^2)", i);
      ImGui::SliderFloat(label, &adcs.manualMagnetorquerMomentAm2[i], -magnetorquers[i]->maxDipoleMoment, magnetorquers[i]->maxDipoleMoment, "%.3f");
    }
  }
}
