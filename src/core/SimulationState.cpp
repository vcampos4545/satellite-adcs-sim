#include "SimulationState.h"
#include "Fsw.h"
#include "Config.h"
#include "rendering/SatelliteRenderer.h"
#include "rendering/OrbitRenderer.h"
#include "rendering/WorldAxesGizmo.h"
#include "panels/GroundStationsPanel.h"
#include "panels/ADCSPanel.h"
#include <rigidbody/orbit/SunModel.h>
#include <rigidbody/orbit/MoonModel.h>
#include <rigidbody/orbit/OrbitFrames.h>
#include <rigidbody/orbit/OrbitTime.h>
#include <rigidbody/orbit/OrbitalElements.h>
#include <rigidbody/orbit/OrbitForceModel.h>

SimulationState::SimulationState(float tumbleKickRadS)
    : camera(1.0f, 0.0f, 0.0f, glm::vec3(0.0f)), // reassigned to real values in buildSimulationState()
      telemetry(Config::TELEMETRY_HISTORY_SAMPLES), simControls(tumbleKickRadS)
{
}

SimulationState buildSimulationState()
{
  SimulationState sim(Config::TUMBLE_KICK_RAD_S);

  sim.spacecraft = buildCubesatPyramid(sim.world, sim.hwConfig);

  sim.earthTexture.loadFromFile(std::string(RESOURCES_DIR) + "/textures/earth.jpg");
  sim.sunTexture.loadFromFile(std::string(RESOURCES_DIR) + "/textures/sun.jpg");
  sim.moonTexture.loadFromFile(std::string(RESOURCES_DIR) + "/textures/moon.jpg");

  sim.camera = OrbitalCamera(Config::CAMERA_INITIAL_DISTANCE, 45.0f, 0.0f, sim.spacecraft.body->position);
  sim.camera.setMaxDistance(Config::CAMERA_MAX_DISTANCE)
      .setMinDistance(Config::CAMERA_MIN_DISTANCE)
      .setZoomSensitivity(Config::ZOOM_SENSITIVITY)
      .setPanSensitivity(Config::PAN_SENSITIVITY);

  // Sun (root) -> Earth -> Moon: the same hierarchy Stage 1's
  // checkPhysicsWorldOrbitalMode() validated end-to-end. Earth/Moon reuse
  // the existing low-precision analytic ephemeris formulas as their
  // parent-relative position (SunModel::positionEci already gives the
  // Sun's position *as seen from Earth*, so Earth's position relative to
  // the Sun is exactly its negation).
  constexpr double GM_SUN = 1.32712440018e20;
  constexpr double GM_MOON = 4.9048695e12;

  CelestialBodyParams sunParams;
  sunParams.mu = GM_SUN;
  sim.sunBody = sim.celestialSystem.addBody("Sun", sunParams);
  sim.celestialSystem.starBody = sim.sunBody;

  CelestialBodyParams earthParams;
  earthParams.mu = TwoBodyGravity{}.mu;
  earthParams.radiusM = OrbitFrames::EARTH_RADIUS_M;
  earthParams.dipoleTiltDeg = 11.0;
  earthParams.dipoleScaleTm3 = 7.94e15;
  sim.earthBody = sim.celestialSystem.addBody("Earth", earthParams, sim.sunBody);
  sim.earthBody->analyticPositionFn = [](double jd) { return -SunModel::positionEci(jd); };

  CelestialBodyParams moonParams;
  moonParams.mu = GM_MOON;
  sim.moonBody = sim.celestialSystem.addBody("Moon", moonParams, sim.earthBody);
  sim.moonBody->analyticPositionFn = [](double jd) { return MoonModel::positionEci(jd); };

  sim.missionEpochJd = OrbitTime::julianDate(sim.epoch.year, sim.epoch.month, sim.epoch.day,
                                              sim.epoch.hour, sim.epoch.minute, sim.epoch.second);
  sim.currentJdNow = sim.missionEpochJd;

  sim.world.attachCelestialSystem(&sim.celestialSystem, sim.missionEpochJd);

  // ISS-like default orbit (500km circular, 51.6deg inclination), perturbed
  // by the Sun and Moon.
  OrbitalElements elements = OrbitalElements::circular(500e3, glm::radians(51.6), OrbitFrames::EARTH_RADIUS_M);
  OrbitState initialState = elements.toState(earthParams.mu);
  sim.world.setOrbitalMode(sim.spacecraft.body, sim.earthBody, {sim.sunBody, sim.moonBody}, initialState);

  sim.orbitPathPoints = computePredictedOrbitPath(sim.world.orbitalState(sim.spacecraft.body), Config::ORBIT_PATH_POINTS, sim.missionEpochJd);
  sim.groundStationPasses = predictGroundStationPasses(
      sim.world.orbitalState(sim.spacecraft.body), sim.missionEpochJd,
      glm::radians(static_cast<double>(Config::GROUND_STATION_MIN_ELEVATION_DEG)),
      Config::PASS_PREDICTION_LOOKAHEAD_S, Config::PASS_PREDICTION_STEP_S);

  sim.magneticFieldLines = traceDipoleFieldLines(sim.fieldLineModel);

  return sim;
}

void SimulationState::step(float dt)
{
  missionEpochJd = OrbitTime::julianDate(epoch.year, epoch.month, epoch.day, epoch.hour, epoch.minute, epoch.second);

  // Ambient field for magnetorquer force generation this cycle is
  // necessarily one cycle stale (last cycle's fieldNow) -- world.step()
  // itself is what propagates the spacecraft to *this* cycle's position,
  // and the field can't be resampled at that new position until after it
  // returns. Same zero-order-hold approximation actuator commands already
  // use; the field changes negligibly over one 20Hz cycle.
  for (auto *rod : spacecraft.magnetorquers)
    rod->ambientFieldWorld = fieldNow;

  world.step(dt);

  const OrbitState &orbitState = world.orbitalState(spacecraft.body);
  currentJdNow = OrbitTime::advance(missionEpochJd, orbitState.missionTimeS);
  inEclipse = world.isInEclipse(spacecraft.body);
  fieldNow = world.ambientFieldAt(spacecraft.body);

  // The rendering/PhysicsWorld frame is Earth-centered (Earth sits at the
  // origin -- it's the spacecraft's orbital-mode primary), not the
  // CelestialSystem's own Sun-rooted frame -- so Sun/Moon must be
  // expressed *relative to Earth* here, not as their absolute (Sun-
  // rooted) positions, or they'd render at/near the origin themselves
  // instead of at their real distance from Earth.
  glm::dvec3 earthAbsPos = celestialSystem.absolutePosition(earthBody, currentJdNow);
  sunPositionNow = glm::vec3(celestialSystem.absolutePosition(sunBody, currentJdNow) - earthAbsPos);
  moonPositionNow = glm::vec3(celestialSystem.absolutePosition(moonBody, currentJdNow) - earthAbsPos);

  orbitPathRefreshTimer += dt;
  if (orbitPathRefreshTimer > Config::ORBIT_PATH_REFRESH_S)
  {
    orbitPathPoints = computePredictedOrbitPath(orbitState, Config::ORBIT_PATH_POINTS, missionEpochJd);
    orbitPathRefreshTimer = 0.0f;
  }

  groundTrackSampleTimer += dt;
  if (groundTrackSampleTimer > Config::GROUND_TRACK_SAMPLE_INTERVAL_S)
  {
    OrbitFrames::Geodetic geo = OrbitFrames::eciToGeodeticDeg(orbitState.position, OrbitFrames::gmstRad(currentJdNow));
    groundTrackLatLonDeg.push_back(glm::vec2(static_cast<float>(geo.latDeg), static_cast<float>(geo.lonDeg)));
    if (static_cast<int>(groundTrackLatLonDeg.size()) > Config::GROUND_TRACK_MAX_POINTS)
      groundTrackLatLonDeg.erase(groundTrackLatLonDeg.begin());
    groundTrackSampleTimer = 0.0f;
  }
}

void SimulationState::refreshGroundStationPasses(float realDt)
{
  groundStationPassRefreshTimer += realDt;
  if (groundStationPassRefreshTimer > Config::PASS_PREDICTION_REFRESH_S)
  {
    groundStationPasses = predictGroundStationPasses(
        world.orbitalState(spacecraft.body), missionEpochJd,
        glm::radians(static_cast<double>(Config::GROUND_STATION_MIN_ELEVATION_DEG)),
        Config::PASS_PREDICTION_LOOKAHEAD_S, Config::PASS_PREDICTION_STEP_S);
    groundStationPassRefreshTimer = 0.0f;
  }
}

void SimulationState::handleCameraInput(GUI &gui, const glm::vec2 &mouseDelta, const glm::vec2 &scrollDelta)
{
  camera.setTarget(spacecraft.body->position);
  if (!ImGui::GetIO().WantCaptureMouse)
    camera.handleInput(gui, mouseDelta, scrollDelta);
  camera.applyToCamera(gui.camera);
}

void SimulationState::draw(GUI &gui, Fsw &fsw, int &selectedPassIndex)
{
  ADCS &adcs = fsw.flightSoftware.adcs;

  // VGL's directional light has no idea where the real Sun is unless told
  // every frame -- see the direction-vs-position distinction this project
  // has always needed here (this must be the direction *toward* the
  // light, not the Sun's position itself).
  gui.setLightDirection(glm::normalize(sunPositionNow));

  if (vis.showEarth)
    drawEarth(gui, earthTexture, currentJdNow);
  if (vis.showSun)
  {
    float sunRadius = static_cast<float>(OrbitFrames::SUN_RADIUS_M);
    if (sunTexture.isLoaded())
      gui.drawTexturedSphere(sunPositionNow, sunRadius, TEXTURED_SPHERE_POLE_ALIGNMENT, sunTexture, /*unlit=*/true);
    else
      gui.drawSphere(sunPositionNow, sunRadius, {1.0f, 0.9f, 0.1f});
  }
  if (vis.showMoon)
  {
    if (moonTexture.isLoaded())
      gui.drawTexturedSphere(moonPositionNow, static_cast<float>(OrbitFrames::MOON_RADIUS_M), TEXTURED_SPHERE_POLE_ALIGNMENT, moonTexture);
    else
      gui.drawSphere(moonPositionNow, static_cast<float>(OrbitFrames::MOON_RADIUS_M), {0.75f, 0.75f, 0.78f});
  }
  if (vis.showOrbitPath)
    drawOrbitPath(gui, orbitPathPoints);
  if (vis.showGroundFootprint)
    drawGroundFootprint(gui, world.orbitalState(spacecraft.body).position, glm::radians(Config::FOOTPRINT_MIN_ELEVATION_DEG));
  if (vis.showGroundStations)
    drawGroundStations(gui, OrbitFrames::gmstRad(currentJdNow));

  if (vis.showSatellite)
  {
    drawSatelliteWireframe(gui, spacecraft.body);
    drawReactionWheels(gui, spacecraft.wheels, spacecraft.body);
    drawMagnetorquers(gui, spacecraft.magnetorquers, spacecraft.body);
    drawMirror(gui, spacecraft.body);
    if (adcs.mode == PointingMode::REFLECT)
      drawSunReflection(gui, spacecraft.body, sunPositionNow);
  }
  if (vis.showMagneticField)
  {
    drawMagneticField(gui, fieldNow, spacecraft.body->position);
    drawMagneticFieldLines(gui, magneticFieldLines);
  }

  if (vis.showTargetMarker)
  {
    gui.drawSphere(adcs.target, Config::TARGET_MARKER_RADIUS_M, {0, 1.0f, 0});
    gui.drawLine(spacecraft.body->position, adcs.target, {0, 1.0f, 0});
  }

  if (vis.showWorldAxesGizmo)
    drawWorldAxesGizmo(gui);

  drawADCSPanel(fsw.flightSoftware, spacecraft, telemetry, simControls, epoch, vis,
                world.orbitalState(spacecraft.body), earthTexture, groundTrackLatLonDeg, currentJdNow,
                groundStationPasses, selectedPassIndex, fsw.trueErrDeg, inEclipse);
}
