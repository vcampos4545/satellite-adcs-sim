#pragma once
#include "Telemetry.h"

// Rolling history of each sensor's reading magnitude, for the Sensors &
// Actuators panel's plots -- one channel per sensor (not per axis) to keep
// the panel compact; per-axis values are still shown as text alongside each
// plot. Pushed once per ADCS cycle (20 Hz, matching when a new reading
// actually exists), not once per render frame.
struct SensorTelemetry
{
  TelemetryChannel gyroMagDegS;
  TelemetryChannel accelMagMs2;
  TelemetryChannel magFieldMagUt;
  TelemetryChannel estimatedPointingErrorDeg; // what the FSW itself computes/would act on
  TelemetryChannel truePointingErrorDeg;      // ground truth, diagnostic only
  TelemetryChannel batterySocPct;
  TelemetryChannel netPowerW; // generation minus consumption -- positive charges, negative discharges

  explicit SensorTelemetry(int samples)
      : gyroMagDegS(samples), accelMagMs2(samples), magFieldMagUt(samples),
        estimatedPointingErrorDeg(samples), truePointingErrorDeg(samples),
        batterySocPct(samples), netPowerW(samples) {}
};
