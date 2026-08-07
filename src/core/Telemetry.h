#pragma once
#include <imgui.h>
#include <cstdio>
#include <cfloat>
#include <algorithm>
#include <vector>

// Shared ImGui telemetry helpers -- scalar-vs-time channels and an XY
// trajectory plot -- used by any scenario that wants a telemetry panel
// instead of window-title text (see examples/starship/launch.cpp and
// examples/starship_landing/landing.cpp). Promoted here once a second
// scenario needed the exact same plotting code, same as World.h.

// Fixed-length rolling history for one scalar channel, drawn as an
// ImGui::PlotLines graph.
struct TelemetryChannel
{
  std::vector<float> samples;
  size_t capacity;

  explicit TelemetryChannel(size_t cap) : capacity(cap) { samples.reserve(cap); }

  void push(float value)
  {
    if (samples.size() >= capacity)
      samples.erase(samples.begin());
    samples.push_back(value);
  }

  float last() const { return samples.empty() ? 0.0f : samples.back(); }
};

inline void plotChannel(const char *label, const TelemetryChannel &ch, const char *unit)
{
  char overlay[64];
  std::snprintf(overlay, sizeof(overlay), "%.1f %s", ch.last(), unit);
  ImGui::PlotLines(label, ch.samples.data(), (int)ch.samples.size(), 0,
                   overlay, FLT_MAX, FLT_MAX, ImVec2(0, 60));
}

// XY trajectory (e.g. downrange vs altitude), unlike TelemetryChannel's
// plots above which are scalar-vs-time. ImGui's core PlotLines only plots a
// single series against an implicit time/index axis, so an actual XY path
// has to be drawn by hand onto the window's draw list. Unbounded (not a
// rolling window) since the point is to see the whole flown path so far --
// even a ~10 minute mission at 60fps is at most a few tens of thousands of
// points, cheap to store and draw.
struct TrajectoryPath
{
  std::vector<ImVec2> points; // x, y in whatever units the caller chooses

  void push(float x, float y)
  {
    points.push_back(ImVec2(x, y));
  }
};

struct TrajectorySeries
{
  const TrajectoryPath *path;
  const char *name;
  ImU32 color;
};

inline void drawTrajectoryPlot(const char *label, const std::vector<TrajectorySeries> &series)
{
  ImGui::Text("%s", label);
  for (size_t i = 0; i < series.size(); i++)
  {
    if (i > 0)
      ImGui::SameLine();
    ImGui::TextColored(ImColor(series[i].color), "-- %s", series[i].name);
  }

  ImVec2 canvasPos = ImGui::GetCursorScreenPos();
  ImVec2 canvasSize(ImGui::GetContentRegionAvail().x, 180.0f);
  ImGui::InvisibleButton(label, canvasSize);

  ImVec2 rectMin = canvasPos;
  ImVec2 rectMax = ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y);
  ImDrawList *drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(rectMin, rectMax, IM_COL32(20, 20, 25, 255));
  drawList->AddRect(rectMin, rectMax, IM_COL32(90, 90, 100, 255));

  // Auto-scaled axes, fit to whichever series currently has the larger
  // range. Always include the origin so the plot reads sensibly before
  // much data has accumulated.
  float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f;
  for (const TrajectorySeries &s : series)
  {
    for (const ImVec2 &pt : s.path->points)
    {
      minX = std::min(minX, pt.x);
      maxX = std::max(maxX, pt.x);
      minY = std::min(minY, pt.y);
      maxY = std::max(maxY, pt.y);
    }
  }
  float rangeX = std::max(maxX - minX, 1.0f);
  float rangeY = std::max(maxY - minY, 1.0f);

  auto toScreen = [&](const ImVec2 &pt)
  {
    float u = (pt.x - minX) / rangeX;
    float v = (pt.y - minY) / rangeY;
    return ImVec2(rectMin.x + u * canvasSize.x, rectMax.y - v * canvasSize.y);
  };

  for (const TrajectorySeries &s : series)
  {
    if (s.path->points.size() < 2)
      continue;
    std::vector<ImVec2> screenPoints;
    screenPoints.reserve(s.path->points.size());
    for (const ImVec2 &pt : s.path->points)
      screenPoints.push_back(toScreen(pt));
    drawList->AddPolyline(screenPoints.data(), (int)screenPoints.size(), s.color, 0, 2.0f);
  }

  char axisLabel[96];
  std::snprintf(axisLabel, sizeof(axisLabel), "x %.0f-%.0f, y %.0f-%.0f", minX, maxX, minY, maxY);
  drawList->AddText(ImVec2(rectMin.x + 4, rectMin.y + 4), IM_COL32(210, 210, 220, 255), axisLabel);
}
