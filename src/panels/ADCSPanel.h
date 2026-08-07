#pragma once
#include "fsw/ADCS.h"
#include "core/Cubesat.h"
#include "core/SensorTelemetry.h"
#include "SimulationPanel.h"
#include "VisualizationPanel.h"
#include "core/GroundStations.h"
#include <rigidbody/orbit/OrbitState.h>
#include <vgl/vgl.h>
#include <vector>

// Single ADCS panel, organized as tabs instead of separate windows so the
// whole flight-software state lives in one place. Tab order is roughly
// most- to least-frequently-needed while iterating on FSW behavior: ADCS
// (pointing mode, attitude algorithm + gains, detumble actuator + B-dot
// gain -- everything that decides *what* to command) first, then Sensors
// (every sensor's current reading plus a rolling plot -- what the FSW
// actually perceives) and Actuators (every actuator's current command +
// health/saturation status, plus manual-override controls) right behind
// it since they're what ADCS reads from/writes to, then FDIR (the
// autonomy layer sitting above all of it) and EPS (a separate subsystem,
// but one FDIR's low-battery fault reads from) as the next tier down,
// then Orbit (background ephemeris context) and Ground Stations (the
// predicted contact schedule that context feeds), and finally the two
// tabs that affect the test harness rather than the spacecraft itself --
// Simulation (time controls/epoch/fault injection, still touched often)
// and Visualization (scene-element toggles, closer to "set once"). See
// FswPanel.h/SensorsPanel.h/ActuatorsPanel.h/FdirPanel.h/EpsPanel.h/
// OrbitPanel.h/GroundStationsPanel.h/SimulationPanel.h/
// VisualizationPanel.h for each tab's own content.
void drawADCSPanel(ADCS &adcs, Cubesat &sat,
                   SensorTelemetry &telemetry, SimControls &sim,
                   EpochControls &epoch, VisualizationSettings &vis,
                   const OrbitState &orbitState, const Texture &earthTexture,
                   const std::vector<glm::vec2> &groundTrack, double currentJd,
                   const std::vector<GroundStationPass> &groundStationPasses, int &selectedPassIndex,
                   float trueErrDeg, bool inEclipse);
