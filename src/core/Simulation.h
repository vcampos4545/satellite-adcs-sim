#pragma once
#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/orbit/CelestialSystem.h>
#include <rigidbody/environment/central_body/CentralBodyMagneticField.h>
#include "Satellite.h"
#include "SensorTelemetry.h"
#include "GroundStations.h"
#include "panels/VisualizationPanel.h"
#include "panels/SimulationPanel.h"
#include "rendering/MagneticFieldRenderer.h"
#include <vector>

class FlightSoftware;

// Everything that defines "the spacecraft and its environment" plus the
// scene/UI state around it -- the physics world, the spacecraft itself,
// the Sun/Earth/Moon hierarchy driving orbital mode, camera/textures, and
// every cached/history quantity the panels and 3D view read. `FlightSoftware`
// (see fsw/FlightSoftware.h) is deliberately kept separate: this class
// never touches FlightSoftware/ADCS itself, only feeds it (see draw()'s
// own FlightSoftware& parameter, used just for readback -- step() lives
// on FlightSoftware).
class Simulation
{
public:
  PhysicsWorld world;
  Satellite spacecraft; // owns its own hwConfig -- see Satellite.h's own comment

  // ECI system
  CelestialSystem celestialSystem;
  CelestialBody *sunBody = nullptr;
  CelestialBody *earthBody = nullptr;
  CelestialBody *moonBody = nullptr;

  OrbitalCamera camera;
  Texture earthTexture, sunTexture, moonTexture;

  SensorTelemetry telemetry;
  VisualizationSettings vis;
  EpochControls epoch;
  SimControls simControls;

  // Cached-for-drawing-while-paused quantities
  glm::vec3 fieldNow{0.0f};
  bool inEclipse = false;
  glm::vec3 sunPositionNow{0.0f};
  glm::vec3 moonPositionNow{0.0f};
  double missionEpochJd = 2451545.0;
  double currentJdNow = 2451545.0;

  std::vector<glm::vec3> orbitPathPoints;
  float orbitPathRefreshTimer = 0.0f;
  std::vector<glm::vec2> groundTrackLatLonDeg;
  float groundTrackSampleTimer = 0.0f;
  std::vector<GroundStationPass> groundStationPasses;
  float groundStationPassRefreshTimer = 0.0f;
  std::vector<FieldLine> magneticFieldLines;
  CentralBodyMagneticField fieldLineModel; // Earth's dipole params, for drawMagneticFieldLines() only

  explicit Simulation();

  // Advances orbital truth + rotational physics by dt (world.step(dt),
  // which drives celestialSystem and the spacecraft's orbital-mode state
  // internally -- see PhysicsWorld::setOrbitalMode()), and refreshes this
  // object's own cached environment/display quantities from the result.
  // Does not touch ADCS/FlightSoftware/ground-station targeting -- see
  // FlightSoftware::step(), which runs after this and reads these results.
  void step(float dt);

  // Reads this cycle's results back off `fsw` (its public telemetry
  // fields -- see FlightSoftware.h's own trueErrDeg/netPowerW comments --
  // plus ADCS's) into `telemetry`'s rolling history for the UI. Called
  // once per FSW cycle (a new sensor reading actually exists), not once
  // per render frame; must run after fsw.step(), which is what actually
  // produces this cycle's values.
  void updateTelemetry(const FlightSoftware &fsw);

  // Wall-clock-timed (not simDt) refresh of the predicted ground-station
  // contact schedule -- called once per render frame regardless of pause
  // state.
  void refreshGroundStationPasses(float realDt);

  void handleCameraInput(GUI &gui, const glm::vec2 &mouseDelta, const glm::vec2 &scrollDelta);

  void draw(GUI &gui, FlightSoftware &fsw, int &selectedPassIndex);
};

Simulation buildSimulation(); // builds the spacecraft, Sun/Earth/Moon hierarchy, textures, camera
