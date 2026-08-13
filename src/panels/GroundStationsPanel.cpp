#include "GroundStationsPanel.h"
#include <rigidbody/orbit/OrbitTime.h>
#include <imgui.h>
#include <cstdio>

namespace
{
// HH:MM:SS (UTC), truncating the fractional second -- AOS/LOS timing
// resolution is already +/- VisualizationConfig::PASS_PREDICTION_STEP_S (a coarser
// figure than sub-second), so displaying fractional seconds would imply
// more precision than the prediction actually has.
void formatClockTime(double jd, char *out, size_t outSize)
{
  int year, month, day, hour, minute;
  double second;
  OrbitTime::calendarDate(jd, year, month, day, hour, minute, second);
  std::snprintf(out, outSize, "%02d:%02d:%02d", hour, minute, static_cast<int>(second));
}

// "10m44s" style duration from a JD span.
void formatDuration(double aosJd, double losJd, char *out, size_t outSize)
{
  double totalS = (losJd - aosJd) * OrbitTime::SECONDS_PER_DAY;
  int minutes = static_cast<int>(totalS) / 60;
  int seconds = static_cast<int>(totalS) % 60;
  std::snprintf(out, outSize, "%dm%02ds", minutes, seconds);
}

// Placeholder for a field this project has no real model for yet --
// shown in disabled/italic-reading style rather than a fabricated number
// (see docs/ALGORITHMS.md's "Ground-station pass prediction" section).
void notYetModeledRow(const char *label)
{
  ImGui::Text("%s", label);
  ImGui::SameLine(180.0f);
  ImGui::TextDisabled("-- not yet modeled");
}
} // namespace

void drawGroundStationsTab(const std::vector<GroundStationPass> &passes, int &selectedIndex)
{
  ImGui::TextDisabled("Predicted contacts over the next 24h -- geometry only (AOS/LOS/max elevation),");
  ImGui::TextDisabled("from the real propagated orbit. Link/weather/scheduling data isn't modeled yet.");

  if (selectedIndex >= static_cast<int>(passes.size()))
    selectedIndex = -1; // the list was refreshed and shrank past the old selection

  ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY;
  if (ImGui::BeginTable("##pass_schedule", 5, tableFlags, ImVec2(0.0f, 220.0f)))
  {
    ImGui::TableSetupColumn("Station");
    ImGui::TableSetupColumn("AOS");
    ImGui::TableSetupColumn("LOS");
    ImGui::TableSetupColumn("Duration");
    ImGui::TableSetupColumn("Max El.");
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableHeadersRow();

    for (int i = 0; i < static_cast<int>(passes.size()); i++)
    {
      const GroundStationPass &pass = passes[i];
      ImGui::TableNextRow();

      ImGui::TableSetColumnIndex(0);
      bool isSelected = (i == selectedIndex);
      char rowId[32];
      std::snprintf(rowId, sizeof(rowId), "%s##row%d", pass.station->name, i);
      if (ImGui::Selectable(rowId, isSelected, ImGuiSelectableFlags_SpanAllColumns))
        selectedIndex = i;

      char aosBuf[16], losBuf[16], durBuf[16];
      formatClockTime(pass.aosJd, aosBuf, sizeof(aosBuf));
      formatClockTime(pass.losJd, losBuf, sizeof(losBuf));
      formatDuration(pass.aosJd, pass.losJd, durBuf, sizeof(durBuf));

      ImGui::TableSetColumnIndex(1);
      ImGui::TextUnformatted(aosBuf);
      ImGui::TableSetColumnIndex(2);
      ImGui::TextUnformatted(losBuf);
      ImGui::TableSetColumnIndex(3);
      ImGui::TextUnformatted(durBuf);
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%.0f deg", pass.maxElevationDeg);
    }
    ImGui::EndTable();
  }

  ImGui::SeparatorText("Pass Detail");
  if (selectedIndex < 0 || passes.empty())
  {
    ImGui::TextDisabled("Select a pass above.");
    return;
  }

  const GroundStationPass &pass = passes[selectedIndex];
  char aosBuf[16], losBuf[16], durBuf[16];
  formatClockTime(pass.aosJd, aosBuf, sizeof(aosBuf));
  formatClockTime(pass.losJd, losBuf, sizeof(losBuf));
  formatDuration(pass.aosJd, pass.losJd, durBuf, sizeof(durBuf));

  const float labelWidth = 180.0f;
  ImGui::Text("Ground station:");
  ImGui::SameLine(labelWidth);
  ImGui::Text("%s", pass.station->name);

  ImGui::Text("Start:");
  ImGui::SameLine(labelWidth);
  ImGui::Text("%s", aosBuf);

  ImGui::Text("End:");
  ImGui::SameLine(labelWidth);
  ImGui::Text("%s", losBuf);

  ImGui::Text("Max elevation:");
  ImGui::SameLine(labelWidth);
  ImGui::Text("%.0f deg", pass.maxElevationDeg);

  ImGui::Text("Duration:");
  ImGui::SameLine(labelWidth);
  ImGui::Text("%s", durBuf);

  notYetModeledRow("Max data rate:");
  notYetModeledRow("Expected data:");
  notYetModeledRow("Link margin:");
  notYetModeledRow("Weather:");
  notYetModeledRow("Priority:");
}
