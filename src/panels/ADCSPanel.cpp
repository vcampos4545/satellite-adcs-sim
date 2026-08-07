#include "ADCSPanel.h"
#include "FswPanel.h"
#include "SensorsPanel.h"
#include "ActuatorsPanel.h"
#include "FdirPanel.h"
#include "EpsPanel.h"
#include "OrbitPanel.h"
#include "GroundStationsPanel.h"
#include "VisualizationPanel.h"
#include <imgui.h>

void drawADCSPanel(FlightSoftware &flightSoftware, Cubesat &sat,
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
  ImGui::Begin("CubeSat ADCS");

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
