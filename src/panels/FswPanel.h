#pragma once
#include "fsw/ADCS.h"
#include "core/SensorTelemetry.h"

// Human-readable names for FSW enums -- shared between the FSW tab itself,
// the FDIR tab's "flying X instead" override message
const char *modeName(PointingMode m);
const char *controllerName(ControllerType c);
const char *detumbleActuatorName(DetumbleActuator a);

// FSW tab: pointing mode selection, pointing-error readout (estimated vs.
// true/ground-truth), the active attitude controller's live-tunable gains,
// and the DETUMBLE actuator selection (reaction wheels vs. B-dot
// magnetorquers) -- everything that decides *what* ADCS commands.
void drawFswTab(ADCS &adcs, SensorTelemetry &telemetry, float trueErrDeg);
