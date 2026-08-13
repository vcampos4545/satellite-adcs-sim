#include "Satellite.h"
#include "SatelliteConfig.h"
#include "PhysicalConstants.h"
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <memory>

namespace
{
  glm::mat3 computeCompositeInertiaTensor()
  {
    using namespace SatelliteConfig;
    // Thin-plate box: Ix = Iy = m*(L^2 + t^2)/12 ~= m*L^2/12 (t << L),
    // Iz = m*(Lx^2 + Ly^2)/12 = m*L^2/6 for a square plate.
    float mirrorIxy = MIRROR_MASS_KG * (MIRROR_SPAN_M * MIRROR_SPAN_M + MIRROR_THICKNESS_M * MIRROR_THICKNESS_M) / 12.0f;
    float mirrorIz = MIRROR_MASS_KG * (MIRROR_SPAN_M * MIRROR_SPAN_M + MIRROR_SPAN_M * MIRROR_SPAN_M) / 12.0f;

    // Cube: I = m*s^2/6, all axes.
    float busI = BUS_MASS_KG * (BUS_CORE_SIDE_M * BUS_CORE_SIDE_M) / 6.0f;

    return glm::mat3(
        mirrorIxy + busI, 0.0f, 0.0f,
        0.0f, mirrorIxy + busI, 0.0f,
        0.0f, 0.0f, mirrorIz + busI);
  }
} // namespace

Satellite buildSatellite(PhysicsWorld &world)
{
  using namespace SatelliteConfig;

  Satellite sat;
  sat.body = world.createBody(
      RigidBodyShape::BOX,
      glm::vec3(MIRROR_SPAN_M, MIRROR_SPAN_M, MIRROR_THICKNESS_M),
      MIRROR_MASS_KG);

  sat.body->setMass(TOTAL_MASS_KG);
  sat.body->setInertiaTensor(computeCompositeInertiaTensor());

  // TODO: Get rid of this, modify in rigid body sim
  sat.body->groundCollisionEnabled = false;

  // IMU board mounted in a corner of the bus, not at the center of mass --
  // like a real PCB, so its accelerometer isn't trivially always-zero (it
  // picks up centripetal/tangential terms from body rotation).
  sat.imu = IMU(IMU_MOUNT_POS);

  // Pyramid layout: 4 wheels, each spin axis tilted `skew` from body +Z,
  // spaced 90 degrees apart in azimuth. Mounted in a small cluster near the
  // +Z face rather than at the body center, matching how a real RWA pyramid
  // bracket is bolted to one panel. Mount offsets scaled up from the old
  // 1U-Satellite's mm-scale mounts to this spacecraft's 0.5m bus core.
  const float skew = glm::radians(WHEEL_PYRAMID_SKEW_DEG);

  for (int i = 0; i < NUM_WHEELS; ++i)
  {
    float azimuth = glm::radians(45.0f) + i * glm::half_pi<float>(); // 45, 135, 225, 315 deg

    glm::vec3 axis(std::sin(skew) * std::cos(azimuth),
                   std::sin(skew) * std::sin(azimuth),
                   std::cos(skew));

    glm::vec3 mountPos(WHEEL_MOUNT_RADIUS_M * std::cos(azimuth),
                       WHEEL_MOUNT_RADIUS_M * std::sin(azimuth),
                       WHEEL_MOUNT_HEIGHT_M);

    float maxSpeedRadS = WHEEL_MAX_SPEED_RPM * (2.0f * glm::pi<float>() / 60.0f);

    auto wheel = std::make_unique<ReactionWheel>(mountPos, axis, WHEEL_MAX_TORQUE_NM, maxSpeedRadS, WHEEL_INERTIA_KGM2);

    sat.wheels.push_back(wheel.get());
    sat.hwConfig.wheels[i] = {axis, WHEEL_MAX_TORQUE_NM, maxSpeedRadS, WHEEL_INERTIA_KGM2};
    sat.body->addForceGenerator(std::move(wheel));
  }

  // Magnetorquer cluster: 3 mutually orthogonal rods along the body axes,
  // the standard Satellite layout (unlike the wheels' skewed pyramid, there's
  // no benefit to tilting a torque rod -- it has no momentum to distribute
  // across axes, so straight body-axis alignment gives the cleanest
  // allocation). Mounts scaled up to the 0.5m bus core; 15 A*m^2 matches a
  // real torque-rod product in this bus class (e.g. ZARM Technik MT15-1).
  const glm::vec3 torquerAxes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  const glm::vec3 torquerMounts[3] = {
      {TORQUER_MOUNT_OFFSET_M, 0, 0}, {0, TORQUER_MOUNT_OFFSET_M, 0}, {0, 0, -TORQUER_MOUNT_OFFSET_M}};
  for (int i = 0; i < NUM_TORQUERS; ++i)
  {
    auto rod = std::make_unique<Magnetorquer>(torquerMounts[i], torquerAxes[i], TORQUER_MAX_MOMENT_AM2);
    sat.magnetorquers.push_back(rod.get());
    sat.hwConfig.torquers[i] = {torquerAxes[i], TORQUER_MAX_MOMENT_AM2};
    sat.body->addForceGenerator(std::move(rod));
  }

  // Magnetometer mounted off-center like the IMU, opposite corner -- real
  // ADCS boards keep the magnetometer away from the torque rods/wheels
  // where practical, since their fields would otherwise swamp the sensor.
  // This model doesn't simulate that interference, but the placement still
  // reflects real layout practice.
  sat.magnetometer = Magnetometer(MAGNETOMETER_MOUNT_POS);

  // Star tracker boresight along body -Z (StarTracker's own default) --
  // opposite the +Z payload/pointing axis every guidance mode here aims,
  // so it isn't staring straight at whatever TARGET/SUN_POINTING/NADIR is
  // currently pointing +Z toward. Real placement follows the same logic:
  // keep the tracker away from the sun-facing/payload side.

  // Solar panels: one body-mounted cell array per face, the standard
  // Satellite layout (vs. a single sun-tracking array) -- whichever face(s)
  // happen to be sunward generate, the rest don't, so generation is a
  // direct function of attitude rather than something a gimbal hides.
  // SOLAR_PANEL_AREA_M2 is a representative deployed-panel dimension (not
  // sat.body->size, which is now the 18m mirror, not the panel envelope):
  // ~1.0m x 1.0m per face, consistent with the mission concept's "~2.5m
  // envelope with panels deployed" bus. ~28% is a representative
  // conversion efficiency for a triple-junction cell (vs. ~20% for
  // cheaper silicon).
  const glm::vec3 panelNormals[6] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const glm::vec3 &n : panelNormals)
    sat.solarPanels.emplace_back(n, SOLAR_PANEL_AREA_M2, SOLAR_PANEL_EFFICIENCY);

  // Battery: a representative Li-ion pack sized up for this bus (40 Wh,
  // vs. the old 1U-Satellite's 10 Wh) -- bigger bus means more housekeeping
  // and actuator draw (see the new wheel/torquer sizing above), while
  // still being small enough that a deliberately harsh test (Simulation
  // tab's "Drain Battery" button, eclipse, or a long run with poor
  // sun-facing geometry) can bring it down to FDIR's low-battery threshold.
  sat.battery = Battery(BATTERY_CAPACITY_WH, BATTERY_MIN_VOLTAGE_V, BATTERY_MAX_VOLTAGE_V, BATTERY_INITIAL_SOC);

  sat.hwConfig.busInertiaTensor = sat.body->inertiaTensor;

  return sat;
}

float Satellite::updatePower(float dt)
{
  float genW = 0.0f;
  if (!inEclipse)
    for (const SolarPanel &panel : solarPanels)
      genW += panel.sample(*body, sunDirWorld, PhysicalConstants::SOLAR_FLUX_WM2).powerW;

  float drawW = SatelliteConfig::POWER_OBC_BASELINE_W + SatelliteConfig::POWER_IMU_W +
                SatelliteConfig::POWER_MAGNETOMETER_W + SatelliteConfig::POWER_STAR_TRACKER_W +
                SatelliteConfig::POWER_SUN_SENSOR_W;
  for (int i = 0; i < NUM_WHEELS; i++)
    drawW += SatelliteConfig::WHEEL_IDLE_POWER_W +
             std::abs(wheels[i]->commandedTorque * wheels[i]->currentSpeed) / SatelliteConfig::WHEEL_MOTOR_EFFICIENCY;
  for (int i = 0; i < NUM_TORQUERS; i++)
    drawW += SatelliteConfig::TORQUER_IDLE_POWER_W +
             std::abs(magnetorquers[i]->commandedDipoleMoment) * SatelliteConfig::TORQUER_POWER_PER_AM2_W;

  float netW = genW - drawW;
  battery.update(netW, dt);
  return netW;
}
