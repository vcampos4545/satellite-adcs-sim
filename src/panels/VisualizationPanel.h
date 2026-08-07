#pragma once

// Which optional 3D-scene elements are currently drawn -- separate from
// simulation state (SimulationPanel.h's SimControls/EpochControls) since
// these only affect what's rendered, never the physics/FSW/orbit
// propagation itself. Held by value in main() and handed to
// drawVisualizationTab() by reference each frame, same pattern as
// SimControls.
//
// showMagneticField defaults to false (everything else defaults on, i.e.
// "as is" relative to this project's behavior before this tab existed):
// the traced dipole field-line set is the densest/most visually heavy
// element in the scene (dozens of multi-segment loops circling the
// globe), so it's opt-in rather than on by default; the local sample
// arrow at the satellite is gated by the same flag since both are "the
// magnetic field" from a user's perspective.
struct VisualizationSettings
{
  bool showEarth = true;
  bool showOrbitPath = true;
  bool showGroundFootprint = true;
  bool showSatellite = true;
  bool showMagneticField = false;
  bool showSun = true;
  bool showMoon = true;
  bool showTargetMarker = true;
  bool showWorldAxesGizmo = true;
};

// Visualization tab: on/off toggles for each optional 3D-scene element,
// so a cluttered view (e.g. the magnetic field lines) can be turned off
// without touching simulation state.
void drawVisualizationTab(VisualizationSettings &vis);
