#pragma once
#include "fsw/ADCS.h"
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Magnetorquer.h>
#include <vector>

// Actuators tab: every actuator's current command + health/saturation
// status, plus the manual-override controls that directly command
// hardware and bypass FSW guidance/control/allocation.
void drawActuatorsTab(ADCS &adcs, const std::vector<ReactionWheel *> &wheels,
                      const std::vector<Magnetorquer *> &magnetorquers);
