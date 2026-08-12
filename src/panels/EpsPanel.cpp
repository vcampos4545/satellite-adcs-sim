#include "EpsPanel.h"
#include "core/Config.h"
#include <imgui.h>

void drawEpsTab(Satellite &sat, ADCS &adcs, SensorTelemetry &telemetry, bool inEclipse)
{
  float soc = sat.battery.stateOfCharge();
  ImVec4 socColor = soc > 0.5f   ? ImVec4(0.3f, 1.0f, 0.4f, 1.0f)
                    : soc > 0.2f ? ImVec4(1.0f, 0.8f, 0.2f, 1.0f)
                                 : ImVec4(1.0f, 0.3f, 0.2f, 1.0f);

  ImGui::SeparatorText("Battery");
  ImGui::TextColored(socColor, "State of charge: %.1f%%", soc * 100.0f);
  plotChannel("State of charge", telemetry.batterySocPct, "%");
  ImGui::Text("Voltage: %.2f V", sat.battery.voltage());
  ImGui::Text("Energy: %.1f / %.1f Wh", sat.battery.energyJ / 3600.0f, sat.battery.capacityJ / 3600.0f);
  ImGui::Text("Net power: %.2f W", telemetry.netPowerW.last());
  plotChannel("Net power", telemetry.netPowerW, "W");
  if (sat.battery.isDepleted())
    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.2f, 1.0f), "DEPLETED");

  ImGui::TextDisabled("Testing:");
  ImGui::SameLine();
  if (ImGui::Button("Drain to 15%"))
    sat.battery.energyJ = 0.15f * sat.battery.capacityJ;
  ImGui::SameLine();
  if (ImGui::Button("Full charge"))
    sat.battery.energyJ = sat.battery.capacityJ;

  ImGui::SeparatorText("Solar Panels");
  glm::vec3 sunDirWorld = adcs.sunPosition - sat.body->position;
  float totalGenW = 0.0f;
  static const char *panelNames[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  for (size_t i = 0; i < sat.solarPanels.size(); i++)
  {
    SolarPanel::Reading r = sat.solarPanels[i].sample(*sat.body, sunDirWorld, Config::SOLAR_FLUX_WM2);
    float panelPowerW = inEclipse ? 0.0f : r.powerW;
    totalGenW += panelPowerW;
    const char *name = i < 6 ? panelNames[i] : "?";
    ImGui::Text("%s face: %5.2f W  (incidence %5.1f deg)", name, panelPowerW, r.incidenceAngleDeg);
  }
  ImGui::Text("Total generation: %.2f W", totalGenW);
  if (inEclipse)
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.7f, 1.0f), "In eclipse -- Earth's shadow, no generation.");
  else
    ImGui::TextDisabled("Sunlit.");
}
