#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/environment/central_body/CentralBodyMagneticField.h>
#include <rigidbody/orbit/OrbitState.h>
#include <rigidbody/orbit/OrbitalElements.h>
#include <rigidbody/orbit/OrbitForceModel.h>
#include <rigidbody/orbit/OrbitPropagator.h>
#include <rigidbody/orbit/OrbitFrames.h>
#include <rigidbody/orbit/OrbitTime.h>
#include <rigidbody/orbit/SunModel.h>
#include <rigidbody/orbit/MoonModel.h>
#include <rigidbody/orbit/EclipseModel.h>
#include <rigidbody/orbit/ThirdBodyGravity.h>
#include <rigidbody/orbit/AtmosphericDrag.h>
#include <rigidbody/orbit/SolarRadiationPressure.h>
#include <rigidbody/power/SolarPanel.h>
#include "fsw/ADCS.h"
#include "fsw/FlightTypes.h"
#include "ImGuiLayer.h"
#include "Config.h"
#include "Cubesat.h"
#include "SensorTelemetry.h"
#include "rendering/SatelliteRenderer.h"
#include "rendering/MagneticFieldRenderer.h"
#include "rendering/OrbitRenderer.h"
#include "rendering/WorldAxesGizmo.h"
#include "panels/SimulationPanel.h"
#include "panels/VisualizationPanel.h"
#include "panels/ADCSPanel.h"
#include "panels/FswPanel.h"
#include <random>
#include <memory>
#include <cmath>
#include <cstdio>

// Random point on Earth's real surface, for an arbitrary ground-pointing
// target (TARGET/SLEW/FINE_POINTING modes). Placed at real Earth radius,
// not an arbitrary unit-sphere point -- now that spacecraftPositionWorld
// is a real ECI position (~6.9e6 m for LEO), a target near the origin
// would both be physically meaningless (inside Earth) and numerically
// broken: `target - spacecraftPositionWorld` with a ~1-unit target and a
// ~6.9e6-unit spacecraft position loses essentially all of target's
// contribution to float32 rounding (catastrophic cancellation) -- the
// direction would come out as just "toward Earth's center" regardless of
// the random target chosen. A target at comparable (Earth-radius) scale
// keeps this subtraction well-conditioned.
static glm::vec3 randomTarget()
{
  static std::mt19937 rng(std::random_device{}());
  static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  glm::vec3 v(dist(rng), dist(rng), dist(rng));
  return glm::normalize(v) * static_cast<float>(OrbitFrames::EARTH_RADIUS_M);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(800, 600, "CubeSat ADCS (Pyramid RWA)");
  ImGuiLayer imguiLayer(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);
  // The scene now spans a 0.1m cubesat up to a ~6.9e6m orbital radius --
  // a standard depth buffer can't hold that dynamic range without
  // z-fighting (near:far is on the order of 1e9). Logarithmic depth
  // trades a small amount of precision at extreme distances for correct
  // ordering across the whole range.
  gui.setLogDepth(Config::CAMERA_FAR);

  // Loaded after the GL context is live (Texture::loadFromFile requires
  // it) -- falls back to a grey 1x1 texture with a printed warning if the
  // file isn't found, so a missing asset doesn't crash the sim.
  Texture earthTexture;
  earthTexture.loadFromFile(std::string(RESOURCES_DIR) + "/textures/earth.jpg");

  PhysicsWorld world;
  HardwareConfig hwConfig;
  Cubesat sat = buildCubesatPyramid(world, hwConfig);

  // Built after the satellite so the orbit target can start on its actual
  // position rather than a magic-number guess at where buildCubesatPyramid()
  // happens to place it.
  OrbitalCamera orbit(
      Config::CAMERA_INITIAL_DISTANCE,
      45.0f, 0.0f,
      sat.body->position);
  orbit.setMaxDistance(Config::CAMERA_MAX_DISTANCE)
      .setMinDistance(Config::CAMERA_MIN_DISTANCE)
      .setZoomSensitivity(Config::ZOOM_SENSITIVITY)
      .setPanSensitivity(Config::PAN_SENSITIVITY);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // Flight software: ADCS is hardware-abstracted (see FlightTypes.h) --
  // configure() gives it a fixed hardware description and an initial
  // attitude estimate; every cycle after this, step() only ever sees
  // plain FSWInputs the harness (this loop) builds from the simulated
  // sensors, and returns plain FSWOutputs the harness applies to the
  // simulated actuators. ADCS itself never holds a RigidBody*/sensor
  // pointer/actuator pointer at all.
  ADCS adcs;
  adcs.configure(hwConfig, sat.body->orientation);
  adcs.target = randomTarget();
  float adcsTimer = 0.0f;
  float missionTime = 0.0f;
  float trueErrDeg = 0.0f; // harness-side diagnostic: true (ground-truth) pointing error, held between ADCS cycles like fieldNow below

  // No Gravity generator is attached to this world (free-floating orbit),
  // so ambient gravity for the IMU model is zero -- this is harness-side
  // simulation setup now, not something ADCS itself ever sees or assumes.
  glm::vec3 gravity{0.0f};

  // Real orbital truth: double-precision ECI state (see rigidbody/orbit/
  // OrbitState.h), propagated independently of sat.body's own float32
  // scene-frame position -- this is genuinely a *different* position, not
  // a rescaled version of it (see FlightTypes.h's spacecraftPositionWorld
  // comment). Two-body + J2, ISS-like default (500km circular, 51.6deg
  // inclination) -- the same altitude/inclination the old kinematic
  // magnetic-field stand-in used, now driving real position, sun
  // direction, eclipse state, and magnetic field.
  OrbitState orbitState = OrbitalElements::circular(
                              500e3, glm::radians(51.6), OrbitFrames::EARTH_RADIUS_M)
                              .toState(TwoBodyGravity{}.mu);
  OrbitPropagator orbitPropagator;
  orbitPropagator.addForceModel(std::make_unique<TwoBodyGravity>());
  orbitPropagator.addForceModel(std::make_unique<J2Perturbation>());
  orbitPropagator.addForceModel(
      std::make_unique<AtmosphericDrag>(Config::SPACECRAFT_CROSS_SECTION_M2, Config::SPACECRAFT_MASS_KG));

  // ThirdBodyGravity/SolarRadiationPressure need epochJd refreshed every
  // frame (see their own header comments) since this project's mission
  // epoch is live-editable from the Simulation tab -- keep raw observing
  // pointers into the propagator's owned models rather than reconstructing
  // them, so OrbitPropagator retains sole ownership.
  auto sunGravityOwned = std::make_unique<ThirdBodyGravity>(ThirdBodyType::Sun);
  auto moonGravityOwned = std::make_unique<ThirdBodyGravity>(ThirdBodyType::Moon);
  auto srpOwned = std::make_unique<SolarRadiationPressure>(Config::SPACECRAFT_CROSS_SECTION_M2, Config::SPACECRAFT_MASS_KG);
  ThirdBodyGravity *sunGravity = sunGravityOwned.get();
  ThirdBodyGravity *moonGravity = moonGravityOwned.get();
  SolarRadiationPressure *srp = srpOwned.get();
  orbitPropagator.addForceModel(std::move(sunGravityOwned));
  orbitPropagator.addForceModel(std::move(moonGravityOwned));
  orbitPropagator.addForceModel(std::move(srpOwned));

  EpochControls epoch; // editable live from the Simulation tab -- see its own comment
  double missionEpochJd = OrbitTime::julianDate(epoch.year, epoch.month, epoch.day, epoch.hour, epoch.minute, epoch.second);
  sunGravity->epochJd = missionEpochJd;
  moonGravity->epochJd = missionEpochJd;
  srp->epochJd = missionEpochJd;

  // Real tilted-dipole field, sampled at orbitState's true position each
  // frame (see below) -- replaces the old MagneticField's fake kinematic
  // orbit phase with the real one. Written into both the magnetorquers
  // (which need it to turn a commanded dipole moment into torque) and
  // ADCS's ambientFieldWorld (its TRIAD reference vector), the same way
  // `gravity` feeds the IMU sample below.
  CentralBodyMagneticField magField;

  // Global field-line geometry for drawMagneticFieldLines -- traced once
  // since this model's dipole is fixed-inertial (see traceDipoleFieldLines'
  // own comment), not per frame.
  std::vector<FieldLine> magneticFieldLines = traceDipoleFieldLines(magField);

  SensorTelemetry telemetry(Config::TELEMETRY_HISTORY_SAMPLES);

  SimControls sim(Config::TUMBLE_KICK_RAD_S);
  VisualizationSettings vis;                // scene-element toggles -- see its own header comment for defaults
  glm::vec3 fieldNow{0.0f};                 // last-sampled ambient field; held while paused rather than resampled
  bool inEclipse = false;                   // last-computed shadow state; held while paused
  glm::vec3 earthRelativePositionNow{0.0f}; // orbitState.position cast to float; held while paused
  glm::vec3 moonPositionNow{0.0f};          // MoonModel::positionEci cast to float; held while paused

  // Predicted orbit path (render-scale points, see computePredictedOrbitPath)
  // -- computed once up front and refreshed periodically, not every frame.
  std::vector<glm::vec3> orbitPathPoints = computePredictedOrbitPath(orbitState, Config::ORBIT_PATH_POINTS, missionEpochJd);
  float orbitPathRefreshTimer = 0.0f;
  double currentJdNow = missionEpochJd; // held while paused, and for drawEarth() below

  // Ground track: (lat, lon) history in degrees, sampled periodically (see
  // Config::GROUND_TRACK_SAMPLE_INTERVAL_S) rather than every frame, capped
  // at Config::GROUND_TRACK_MAX_POINTS so it spans multiple orbits without
  // growing unbounded over a long-running mission.
  std::vector<glm::vec2> groundTrackLatLonDeg;
  float groundTrackSampleTimer = 0.0f;

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;

    // =================== INPUT ===================
    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;

    if (gui.isKeyJustPressed(GLFW_KEY_1))
      adcs.mode = PointingMode::NADIR;
    if (gui.isKeyJustPressed(GLFW_KEY_2))
      adcs.mode = PointingMode::SUN_POINTING;
    if (gui.isKeyJustPressed(GLFW_KEY_3))
      adcs.mode = PointingMode::DETUMBLE;
    if (gui.isKeyJustPressed(GLFW_KEY_4))
      adcs.mode = PointingMode::TARGET;
    if (gui.isKeyJustPressed(GLFW_KEY_5))
      adcs.mode = PointingMode::SLEW;
    if (gui.isKeyJustPressed(GLFW_KEY_6))
      adcs.mode = PointingMode::FINE_POINTING;
    if (gui.isKeyJustPressed(GLFW_KEY_7))
      adcs.mode = PointingMode::REFLECT;

    if (gui.isKeyJustPressed(GLFW_KEY_SPACE))
    {
      adcs.target = randomTarget();
      adcs.resetController(); // clear integral windup from previous target
    }

    if (gui.isKeyJustPressed(GLFW_KEY_T))
    {
      static std::mt19937 tumbleRng(std::random_device{}());
      std::uniform_real_distribution<float> tumbleDist(-sim.tumbleKickRadS, sim.tumbleKickRadS);
      sat.body->angularVelocity = glm::vec3(tumbleDist(tumbleRng), tumbleDist(tumbleRng), tumbleDist(tumbleRng));
    }

    // Follow the satellite: recenter the orbit camera's target on its
    // real current position every frame (like constellation-sim does for
    // a selected satellite), so free yaw/pitch/zoom around it still work
    // (setTarget doesn't touch distance/yaw/pitch) while any right-drag
    // pan gets overridden next frame -- a locked-on chase camera can't
    // also be panned away from what it's locked onto.
    orbit.setTarget(sat.body->position);

    // Don't drive the orbit camera from mouse input ImGui itself wants
    // (e.g. dragging the Controller window around) -- otherwise moving a
    // panel also spins the camera underneath it.
    if (!ImGui::GetIO().WantCaptureMouse)
      orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // =================== SIMULATION ===================
    // Everything below (mission clock, field sampling, FSW, fault
    // injection, physics) freezes while paused; camera/UI/mode selection
    // above stay live so the panel is still usable mid-pause.
    //
    // Time controls (see SimulationPanel.h's SimControls comment): while
    // running, `simDt` is real dt scaled by sim.timeScale, so this whole
    // block (and everything inside it) advances *simulated* time at that
    // rate rather than 1:1 with wall clock. While paused, a single
    // Config::TIME_STEP_S step runs instead if the Simulation tab's Step
    // button set stepRequested -- a one-shot request, cleared immediately
    // so holding the pause checkbox doesn't repeatedly re-trigger it.
    bool advanceSim = !sim.paused;
    float simDt = dt * sim.timeScale;
    if (sim.paused && sim.stepRequested)
    {
      advanceSim = true;
      simDt = Config::TIME_STEP_S;
      sim.stepRequested = false;
    }

    if (advanceSim)
    {
      missionTime += simDt;

      // Real orbital truth, propagated every frame at simDt (RK4 is
      // stable at any step size -- smaller just means more calls, not
      // less accuracy; larger, from a high timeScale, trades a little
      // accuracy for speed, matching this project's existing debug-tool
      // posture) -- integrated independently of PhysicsWorld::step()
      // below in double precision, then bridged into sat.body->position
      // as a single non-accumulating cast every frame (see
      // OrbitState.h's header comment for why the double-precision
      // integration itself has to stay separate from RigidBody's float32
      // state, even though the *result* is copied into it here). Because
      // this is a fresh cast from the true state each frame rather than
      // something PhysicsWorld itself integrates, there's no float32
      // accumulation risk -- sat.body->velocity is deliberately never set
      // to match, so PhysicsWorld's own (float32) translational
      // integration contributes nothing on top of this.
      orbitPropagator.step(orbitState, simDt);
      // Recomputed from the Simulation tab's live-editable fields every
      // frame -- cheap (six ints/one float into a JD formula), and means
      // an edit takes effect immediately without a separate "apply" step.
      missionEpochJd = OrbitTime::julianDate(epoch.year, epoch.month, epoch.day, epoch.hour, epoch.minute, epoch.second);
      sunGravity->epochJd = missionEpochJd;
      moonGravity->epochJd = missionEpochJd;
      srp->epochJd = missionEpochJd;
      currentJdNow = OrbitTime::advance(missionEpochJd, orbitState.missionTimeS);
      earthRelativePositionNow = glm::vec3(orbitState.position); // single non-accumulating cast -- see OrbitState.h
      sat.body->position = earthRelativePositionNow;
      glm::dvec3 sunDirEci = SunModel::directionEci(currentJdNow);
      inEclipse = EclipseModel::inEclipse(orbitState.position, sunDirEci);
      moonPositionNow = glm::vec3(MoonModel::positionEci(currentJdNow)); // single non-accumulating cast, same as earthRelativePositionNow

      // Refresh the predicted-path polyline periodically (not every
      // frame -- see ORBIT_PATH_REFRESH_S) since it only drifts slowly
      // (mainly from J2) cycle to cycle.
      orbitPathRefreshTimer += simDt;
      if (orbitPathRefreshTimer > Config::ORBIT_PATH_REFRESH_S)
      {
        orbitPathPoints = computePredictedOrbitPath(orbitState, Config::ORBIT_PATH_POINTS, missionEpochJd);
        orbitPathRefreshTimer = 0.0f;
      }

      // Sample the ground track periodically (not every frame -- see
      // GROUND_TRACK_SAMPLE_INTERVAL_S).
      groundTrackSampleTimer += simDt;
      if (groundTrackSampleTimer > Config::GROUND_TRACK_SAMPLE_INTERVAL_S)
      {
        OrbitFrames::Geodetic geo = OrbitFrames::eciToGeodeticDeg(orbitState.position, OrbitFrames::gmstRad(currentJdNow));
        groundTrackLatLonDeg.push_back(glm::vec2(static_cast<float>(geo.latDeg), static_cast<float>(geo.lonDeg)));
        if (static_cast<int>(groundTrackLatLonDeg.size()) > Config::GROUND_TRACK_MAX_POINTS)
          groundTrackLatLonDeg.erase(groundTrackLatLonDeg.begin());
        groundTrackSampleTimer = 0.0f;
      }

      // Sun position for guidance/EPS: the real ECI position (~1.5e11 m),
      // not an arbitrary nearby offset -- now that sat.body->position is
      // real too (~6.9e6 m), placing the sun a small fixed distance away
      // (the old "2 units from the satellite" convention) would suffer
      // the same catastrophic-cancellation problem randomTarget()'s
      // comment describes: adding a small offset to spacecraftPositionWorld
      // and then subtracting it back out to recover a direction loses
      // almost all of that offset's precision once the base position's
      // magnitude dwarfs it. The real Sun position doesn't have this
      // problem -- it's the *dominant* term in sunPosition -
      // spacecraftPositionWorld, not a small perturbation of it, so the
      // subtraction stays well-conditioned.
      adcs.sunPosition = glm::vec3(SunModel::positionEci(currentJdNow));

      // Ambient field at the satellite's real orbital position -- fed to
      // the magnetorquers (they need it every physics substep to turn a
      // commanded dipole moment into torque) and to ADCS (it needs it to
      // interpret the magnetometer), same role adcs.gravity plays for the
      // IMU.
      fieldNow = magField.sample(earthRelativePositionNow);
      for (auto *rod : sat.magnetorquers)
        rod->ambientFieldWorld = fieldNow;
      adcs.ambientFieldWorld = fieldNow;

      // =================== FLIGHT SOFTWARE (20 Hz) ===================
      // This is the HAL boundary in action: sample every simulated sensor,
      // pack the readings into a plain FSWInputs, hand it to ADCS::step()
      // (which never sees a RigidBody/sensor/actuator object at all), then
      // apply the plain FSWOutputs it returns to the simulated actuators.
      // A HIL adapter would replace exactly the sampling and command-
      // application on either side of that call -- ADCS::step() itself
      // wouldn't change.
      //
      // A `while`, not an `if`: at timeScale > 1x, simDt can span more
      // than one nominal Config::TIME_STEP_S cycle in a single render
      // frame -- an `if` would silently run the FSW cycle at a *slower
      // relative rate* the faster time is scaled (fewer corrections per
      // orbit), which would visibly degrade pointing/detumble stability
      // at high speeds for no physical reason. Catching up with a fixed-
      // size `while` keeps the FSW cadence correct relative to simulated
      // time regardless of timeScale, the same fixed-step-accumulator
      // pattern PhysicsWorld::step() already uses internally.
      adcsTimer += simDt;
      while (adcsTimer > Config::TIME_STEP_S)
      {
        adcsTimer -= Config::TIME_STEP_S;

        IMU::Reading imuReading = sat.imu.sample(*sat.body, gravity, Config::TIME_STEP_S);
        Magnetometer::Reading magReading = sat.magnetometer.sample(*sat.body, fieldNow, Config::TIME_STEP_S);
        glm::vec3 sunDirWorld = adcs.sunPosition - sat.body->position;
        StarTracker::Reading starReading = sat.starTracker.sample(*sat.body, sunDirWorld);
        SunSensor::Reading sunReading = sat.sunSensor.sample(*sat.body, sunDirWorld);

        FSWInputs inputs;
        inputs.imu = {imuReading.gyro, imuReading.accel};
        inputs.mag = {magReading.field, true};
        inputs.star = {starReading.attitude, starReading.valid};
        inputs.sunSensor = {sunReading.sunDirBody, sunReading.valid};
        // Last-known EPS telemetry -- this cycle's own consumption (wheels/
        // torquers, computed below from what step() is about to command)
        // hasn't happened yet, same "read before this cycle's effects"
        // relationship inputs.wheelTelemetry[i].speedRadS already has with
        // the wheel commands step() is about to issue.
        inputs.power = {sat.battery.stateOfCharge(), sat.battery.voltage()};
        for (int i = 0; i < NUM_WHEELS; i++)
          inputs.wheelTelemetry[i] = {sat.wheels[i]->currentSpeed, sat.wheels[i]->healthFactor > 0.01f};
        inputs.spacecraftPositionWorld = sat.body->position; // real orbital position (ECI meters) -- stands in for a real nav solution

        FSWOutputs out = adcs.step(inputs, Config::TIME_STEP_S);

        for (int i = 0; i < NUM_WHEELS; i++)
          sat.wheels[i]->commandTorque(out.wheelCommands[i].torqueNm);
        for (int i = 0; i < NUM_TORQUERS; i++)
          sat.magnetorquers[i]->commandDipoleMoment(out.torquerCommands[i].momentAm2);

        // =================== EPS (same 20 Hz cycle) ===================
        // Generation: sum every panel's cosine-law output against the same
        // sun direction the star tracker/sun sensor were just sampled
        // against -- zero while the real orbital position is in Earth's
        // shadow (see EclipseModel::inEclipse above), closing this
        // project's former "no orbital eclipse model" gap. Consumption: a
        // fixed housekeeping/sensor draw plus each actuator's
        // idle-plus-effort power for the commands just issued above -- see
        // the Config::POWER_* comments for the model each term follows.
        // Net power integrates straight into the battery.
        float genW = 0.0f;
        if (!inEclipse)
          for (const SolarPanel &panel : sat.solarPanels)
            genW += panel.sample(*sat.body, sunDirWorld, Config::SOLAR_FLUX_WM2).powerW;

        float drawW = Config::POWER_OBC_BASELINE_W + Config::POWER_IMU_W +
                      Config::POWER_MAGNETOMETER_W + Config::POWER_STAR_TRACKER_W +
                      Config::POWER_SUN_SENSOR_W;
        for (int i = 0; i < NUM_WHEELS; i++)
          drawW += Config::WHEEL_IDLE_POWER_W +
                   std::abs(out.wheelCommands[i].torqueNm * sat.wheels[i]->currentSpeed) / Config::WHEEL_MOTOR_EFFICIENCY;
        for (int i = 0; i < NUM_TORQUERS; i++)
          drawW += Config::TORQUER_IDLE_POWER_W +
                   std::abs(out.torquerCommands[i].momentAm2) * Config::TORQUER_POWER_PER_AM2_W;

        sat.battery.update(genW - drawW, Config::TIME_STEP_S);
        telemetry.netPowerW.push(genW - drawW);

        // Ground-truth diagnostic -- computed here, not by ADCS (which has
        // no way to know body->orientation at all anymore), the same way
        // this project's earlier true-vs-estimated pointing error
        // comparisons always required direct simulation access.
        glm::quat trueErrQ = glm::inverse(sat.body->orientation) * adcs.targetAttitude;
        if (trueErrQ.w < 0.0f)
          trueErrQ = -trueErrQ;
        trueErrDeg = glm::degrees(2.0f * std::acos(glm::clamp(trueErrQ.w, -1.0f, 1.0f)));

        // Pushed once per ADCS cycle (a new sensor reading actually
        // exists), not once per render frame.
        telemetry.gyroMagDegS.push(glm::degrees(glm::length(adcs.lastGyroBody)));
        telemetry.accelMagMs2.push(glm::length(adcs.lastAccelBody));
        telemetry.magFieldMagUt.push(glm::length(adcs.magFieldBody) * 1e6f);
        telemetry.estimatedPointingErrorDeg.push(adcs.estimatedPointingErrorDeg);
        telemetry.truePointingErrorDeg.push(trueErrDeg);
        telemetry.batterySocPct.push(sat.battery.stateOfCharge() * 100.0f);
      }

      // =================== PHYSICS ===================
      world.step(simDt);
    }

    // =================== DRAW ===================
    // VGL's scene lighting is a single directional light (see GUI.h's
    // m_lightDir / EmbeddedShaders.h's diffuse term), defaulting to an
    // arbitrary fixed world-space direction -- it has no idea where the
    // real Sun is unless told every frame. `lightDir` is the direction
    // *toward* the light (the shader does `dot(normal, normalize(lightDir))`),
    // so this must be the direction from Earth's center toward the Sun,
    // not the Sun's position itself -- without this call Earth's lit
    // hemisphere is disconnected from where the Sun marker is actually
    // drawn, which is what made the sunlit side look inverted.
    gui.setLightDirection(glm::normalize(adcs.sunPosition));
    gui.beginFrame();
    imguiLayer.beginFrame();

    // =================== ORBIT VISUALIZATION ===================
    // Earth and the predicted orbit path, both in real ECI meters --
    // PhysicsWorld's own frame here *is* ECI (Earth's center at the world
    // origin, sat.body->position bridged from orbitState every frame
    // below), so the satellite's own wireframe/wheels/rods/etc. (drawn
    // further down, already reading sat.body->position/orientation) are
    // already at the right place -- no separate marker needed.
    if (vis.showEarth)
      drawEarth(gui, earthTexture, currentJdNow);
    if (vis.showOrbitPath)
      drawOrbitPath(gui, orbitPathPoints);
    if (vis.showGroundFootprint)
      drawGroundFootprint(gui, orbitState.position, glm::radians(Config::FOOTPRINT_MIN_ELEVATION_DEG));

    if (vis.showSatellite)
    {
      drawSatelliteWireframe(gui, sat.body);
      drawReactionWheels(gui, sat.wheels, sat.body);
      drawMagnetorquers(gui, sat.magnetorquers, sat.body);
      drawMirror(gui, sat.body);
      if (adcs.mode == PointingMode::REFLECT)
        drawSunReflection(gui, sat.body, adcs.sunPosition);
    }
    if (vis.showMagneticField)
    {
      drawMagneticField(gui, fieldNow, sat.body->position);
      drawMagneticFieldLines(gui, magneticFieldLines);
    }

    if (vis.showTargetMarker)
      gui.drawSphere(adcs.target, Config::TARGET_MARKER_RADIUS_M, {0, 1.0f, 0});

    if (vis.showSun)
    {
      // Sun marker sized to its *real* angular diameter (~32 arcmin) at
      // its real current distance from the satellite (~1 AU), rather
      // than a fixed prop radius -- r = d*tan(halfAngle).
      float sunDistance = glm::length(adcs.sunPosition - sat.body->position);
      float sunRadius = sunDistance * std::tan(glm::radians(Config::SUN_ANGULAR_DIAMETER_DEG * 0.5f));
      gui.drawSphere(adcs.sunPosition, sunRadius, {1.0f, 0.9f, 0.1f});
    }

    if (vis.showMoon)
    {
      // Moon sphere at its real ECI position/size (MoonModel/OrbitFrames::
      // MOON_RADIUS_M) -- unlike the Sun, close enough (~3.84e8m vs. the
      // Sun's ~1.5e11m) that its real radius alone (not an angular-diameter
      // trick) renders it as a visible disk at the same real distance. No
      // texture asset for the Moon (unlike Earth) -- a plain lit gray sphere
      // is enough to show its real position/motion.
      gui.drawSphere(moonPositionNow, static_cast<float>(OrbitFrames::MOON_RADIUS_M), {0.75f, 0.75f, 0.78f});
    }

    if (vis.showTargetMarker)
    {
      // Pointing-error visualization: a line from the body straight to
      // the target makes the *angular gap* between where the body
      // actually points and where it should legible at a glance -- much
      // easier to judge by eye than comparing the wireframe's own +Z
      // arrow (drawn in drawSatelliteWireframe) against a distant marker.
      gui.drawLine(sat.body->position, adcs.target, {0, 1.0f, 0});
    }

    if (vis.showWorldAxesGizmo)
      drawWorldAxesGizmo(gui);

    drawADCSPanel(adcs, sat, telemetry, sim, epoch, vis, orbitState, earthTexture, groundTrackLatLonDeg, currentJdNow, trueErrDeg, inEclipse);

    imguiLayer.endFrame();
    gui.endFrame();
  }
  return 0;
}
