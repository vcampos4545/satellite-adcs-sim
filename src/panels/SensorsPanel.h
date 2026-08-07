#pragma once
#include "fsw/ADCS.h"
#include "core/SensorTelemetry.h"

// Sensors tab: every sensor's current reading plus a rolling plot -- what
// the FSW actually perceives (as opposed to Actuators/EPS, which show
// what's actually happening on the bus).
void drawSensorsTab(ADCS &adcs, SensorTelemetry &telemetry);
