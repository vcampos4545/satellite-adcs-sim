#include "VisualizationPanel.h"
#include <imgui.h>

void drawVisualizationTab(VisualizationSettings &vis)
{
  ImGui::SeparatorText("Scene Elements");
  ImGui::TextDisabled("Display only -- none of these affect physics, FSW, or orbit propagation.");
  ImGui::Checkbox("Earth", &vis.showEarth);
  ImGui::Checkbox("Orbit path (predicted)", &vis.showOrbitPath);
  ImGui::Checkbox("Ground footprint", &vis.showGroundFootprint);
  ImGui::Checkbox("Satellite (wireframe/wheels/torquers/mirror)", &vis.showSatellite);
  ImGui::Checkbox("Magnetic field (local vector + traced dipole lines)", &vis.showMagneticField);
  ImGui::TextDisabled("Off by default -- the traced field lines are the densest element in the scene.");
  ImGui::Checkbox("Sun", &vis.showSun);
  ImGui::Checkbox("Moon", &vis.showMoon);
  ImGui::Checkbox("Target marker/line", &vis.showTargetMarker);
  ImGui::Checkbox("World axes gizmo", &vis.showWorldAxesGizmo);
}
