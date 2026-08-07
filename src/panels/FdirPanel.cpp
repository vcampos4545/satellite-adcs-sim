#include "FdirPanel.h"
#include <imgui.h>
#include <cstring>
#include <cstdio>

static const char *fdirFaultName(uint32_t flag)
{
  switch (flag)
  {
  case FDIR_FAULT_WHEEL_AUTHORITY_LOST:
    return "Wheel authority lost";
  case FDIR_FAULT_ATTITUDE_UNCERTAIN:
    return "Attitude uncertain";
  case FDIR_FAULT_EXCESS_RATE:
    return "Excess body rate";
  case FDIR_FAULT_LOW_BATTERY:
    return "Low battery";
  default:
    return "Unknown";
  }
}

// Renders `flags` as a comma-separated list of fault names into `out`
// (fixed-size, matching this project's no-dynamic-allocation-in-FSW-UI
// convention already used elsewhere in this file for label buffers).
static void formatFaultFlags(uint32_t flags, char *out, size_t outSize)
{
  out[0] = '\0';
  bool first = true;
  for (uint32_t bit = 1; bit != 0; bit <<= 1)
  {
    if (!(flags & bit))
      continue;
    size_t used = std::strlen(out);
    std::snprintf(out + used, outSize - used, "%s%s", first ? "" : ", ", fdirFaultName(bit));
    first = false;
  }
}

void drawFdirTab(ADCS &adcs)
{
  FDIR &fdir = adcs.fdir;

  ImGui::SeparatorText("Status");
  if (fdir.state() == FdirState::NOMINAL)
    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.4f, 1.0f), "NOMINAL");
  else
    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.2f, 1.0f), "SAFE_HOLD");

  ImGui::Checkbox("Autonomy enabled", &fdir.enabled);
  ImGui::TextDisabled("When off, faults are still detected and logged but never override the commanded mode.");

  ImGui::SeparatorText("Active Faults");
  uint32_t active = fdir.activeFaults();
  if (active == FDIR_FAULT_NONE)
  {
    ImGui::TextDisabled("None");
  }
  else
  {
    for (uint32_t bit = 1; bit != 0; bit <<= 1)
      if (active & bit)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "[%s]", fdirFaultName(bit));
  }
  if (ImGui::Button("Clear latched faults (ground command)"))
    fdir.clearLatchedFaults();
  ImGui::TextDisabled("Faults latch on trip and stay active even if the condition clears -- this is the explicit ack that drops them.");

  ImGui::SeparatorText("Thresholds");
  ImGui::DragInt("Min healthy wheels", &fdir.minHealthyWheels, 0.05f, 0, NUM_WHEELS);
  ImGui::DragFloat("Uncertainty trigger (deg)", &fdir.attitudeUncertaintyTriggerDeg, 0.1f, 0.1f, 45.0f, "%.1f");
  ImGui::DragFloat("Uncertainty sustain (s)", &fdir.attitudeUncertaintySustainedS, 0.5f, 0.0f, 60.0f, "%.1f");
  ImGui::DragFloat("Excess rate (rad/s)", &fdir.excessRateRadS, 0.05f, 0.1f, 10.0f, "%.2f");
  ImGui::DragFloat("Low battery trigger (fraction)", &fdir.lowBatterySocTrigger, 0.01f, 0.0f, 1.0f, "%.2f");

  ImGui::SeparatorText("Event Log (newest first)");
  if (fdir.eventCount == 0)
    ImGui::TextDisabled("No events yet.");
  for (int i = 0; i < fdir.eventCount; i++)
  {
    const FdirEvent &ev = fdir.recentEvent(i);
    char flagsBuf[128];
    formatFaultFlags(ev.flags, flagsBuf, sizeof(flagsBuf));
    ImVec4 color = ev.entering ? ImVec4(1.0f, 0.4f, 0.2f, 1.0f) : ImVec4(0.5f, 0.8f, 1.0f, 1.0f);
    ImGui::TextColored(color, "[t=%7.1fs] %s: %s", ev.missionTimeS, ev.entering ? "TRIPPED" : "CLEARED", flagsBuf);
  }
}
