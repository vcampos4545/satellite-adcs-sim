#include "ADCSPanel.h"
#include "FswPanel.h"
#include "SensorsPanel.h"
#include "ActuatorsPanel.h"
#include "FdirPanel.h"
#include "EpsPanel.h"
#include "OrbitPanel.h"
#include "GroundStationsPanel.h"
#include "VisualizationPanel.h"
#include <rigidbody/orbit/OrbitTime.h>
#include <imgui.h>

void drawADCSPanel(FlightSoftware &flightSoftware, Satellite &sat,
                   SensorTelemetry &telemetry, SimControls &sim,
                   EpochControls &epoch, VisualizationSettings &vis,
                   const OrbitState &orbitState, const Texture &earthTexture,
                   const std::vector<glm::vec2> &groundTrack, double currentJd,
                   const std::vector<GroundStationPass> &groundStationPasses, int &selectedPassIndex,
                   float trueErrDeg, bool inEclipse)
{
  ADCS &adcs = flightSoftware.adcs;

  ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(380, 560), ImGuiCond_FirstUseEver);
  ImGui::Begin("Satellite ADCS");

  // Persistent header, above the tab bar so it's visible regardless of
  // which tab is selected -- current mission clock (UTC, derived from
  // currentJd) and elapsed mission time (orbitState.missionTimeS, seconds
  // since EpochControls' commanded epoch -- see OrbitTime.h), the same two
  // clocks the rest of this panel's orbital/ground-station math runs on.
  {
    int year, month, day, hour, minute;
    double second;
    OrbitTime::calendarDate(currentJd, year, month, day, hour, minute, second);

    double missionTimeS = orbitState.missionTimeS;
    int missionDays = static_cast<int>(missionTimeS / 86400.0);
    double remS = missionTimeS - missionDays * 86400.0;
    int missionHours = static_cast<int>(remS / 3600.0);
    remS -= missionHours * 3600.0;
    int missionMinutes = static_cast<int>(remS / 60.0);
    remS -= missionMinutes * 60.0;

    ImGui::Text("%04d-%02d-%02d %02d:%02d:%02.0f UTC", year, month, day, hour, minute, second);
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    ImGui::Text("Mission Time: %dd %02dh %02dm %02.0fs", missionDays, missionHours, missionMinutes, remS);
    ImGui::Separator();
  }

  if (ImGui::BeginTabBar("ADCSTabs"))
  {
    if (ImGui::BeginTabItem("ADCS"))
    {
      drawFswTab(adcs, telemetry, trueErrDeg);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Sensors"))
    {
      drawSensorsTab(adcs, telemetry);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Actuators"))
    {
      drawActuatorsTab(adcs, sat.wheels, sat.magnetorquers);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("FDIR"))
    {
      drawFdirTab(flightSoftware.fdir);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("EPS"))
    {
      drawEpsTab(sat, adcs, telemetry, inEclipse);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Orbit"))
    {
      drawOrbitTab(orbitState, earthTexture, groundTrack, currentJd);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Ground Stations"))
    {
      drawGroundStationsTab(groundStationPasses, selectedPassIndex);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Simulation"))
    {
      drawSimulationTab(sim, epoch, sat.wheels, sat.body, adcs);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Visualization"))
    {
      drawVisualizationTab(vis);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  ImGui::End();
}
