#include "SimFlightSoftwareHAL.h"

SimFlightSoftwareHAL::SimFlightSoftwareHAL(Cubesat &sat) : sat_(sat) {}

void SimFlightSoftwareHAL::setEnvironment(const glm::vec3 &gravity, const glm::vec3 &ambientFieldWorld,
                                           const glm::vec3 &sunDirWorld, bool inEclipse)
{
  gravity_ = gravity;
  ambientFieldWorld_ = ambientFieldWorld;
  sunDirWorld_ = sunDirWorld;
  inEclipse_ = inEclipse;
}

ImuSample SimFlightSoftwareHAL::readImu(float dt)
{
  IMU::Reading r = sat_.imu.sample(*sat_.body, gravity_, dt);
  return {r.gyro, r.accel};
}

MagSample SimFlightSoftwareHAL::readMag(float dt)
{
  Magnetometer::Reading r = sat_.magnetometer.sample(*sat_.body, ambientFieldWorld_, dt);
  return {r.field, true};
}

StarTrackerSample SimFlightSoftwareHAL::readStarTracker()
{
  StarTracker::Reading r = sat_.starTracker.sample(*sat_.body, sunDirWorld_);
  return {r.attitude, r.valid};
}

SunSensorSample SimFlightSoftwareHAL::readSunSensor()
{
  SunSensor::Reading r = sat_.sunSensor.sample(*sat_.body, sunDirWorld_);
  // SunSensor has no eclipse model of its own (see its header: "valid is
  // always true here... a scenario that wants eclipse-aware dropouts needs
  // to gate this externally") -- a real coarse sun sensor reports no lock
  // when the sun itself is physically blocked by Earth, not just when the
  // geometric direction happens to be undefined, so this HAL applies that
  // blindness here. This also keeps TRIAD fallback (computeTriadFallback,
  // ADCS.cpp) honest during eclipse: it already requires in.sunSensor.valid,
  // but without this gate it would happily TRIAD off a sun direction the
  // satellite couldn't actually observe.
  bool valid = r.valid && !inEclipse_;
  return {r.sunDirBody, valid};
}

PowerSample SimFlightSoftwareHAL::readPower()
{
  return {sat_.battery.stateOfCharge(), sat_.battery.voltage()};
}

std::array<WheelTelemetry, NUM_WHEELS> SimFlightSoftwareHAL::readWheelTelemetry()
{
  std::array<WheelTelemetry, NUM_WHEELS> out{};
  for (int i = 0; i < NUM_WHEELS; i++)
    out[i] = {sat_.wheels[i]->currentSpeed, sat_.wheels[i]->healthFactor > 0.01f};
  return out;
}

glm::vec3 SimFlightSoftwareHAL::readNavPosition()
{
  return sat_.body->position; // real orbital position (ECI meters) -- stands in for a real nav solution
}

void SimFlightSoftwareHAL::commandWheelTorque(int index, float torqueNm)
{
  sat_.wheels[index]->commandTorque(torqueNm);
  lastWheelTorqueNm[index] = torqueNm;
}

void SimFlightSoftwareHAL::commandTorquerMoment(int index, float momentAm2)
{
  sat_.magnetorquers[index]->commandDipoleMoment(momentAm2);
  lastTorquerMomentAm2[index] = momentAm2;
}
