#pragma once
#include "fsw/ADCS.h"
#include "core/Cubesat.h"
#include "core/SensorTelemetry.h"

// EPS (electrical power subsystem) tab: battery state, live solar
// generation per panel, and the same net-power/SOC plots the FSW-cycle
// telemetry is pushed to -- the electrical equivalent of the Sensors/
// Actuators tabs, showing what's actually happening on the bus rather
// than what FSW perceives (there's no EPS "sensor model" with its own
// noise/dropouts in this project -- battery telemetry is read directly,
// see PowerSample's header comment).
void drawEpsTab(Cubesat &sat, ADCS &adcs, SensorTelemetry &telemetry, bool inEclipse);
