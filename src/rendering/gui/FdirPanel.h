#pragma once
#include "fsw/FDIR.h"

// Autonomous mode-manager/FDIR status: what state it's in, which faults are
// currently latched, the tunable thresholds that decide when a fault trips,
// and a scrollable log of every trip/clear event -- the flight-software
// equivalent of a fault log a real ops team would review after the fact.
void drawFdirTab(FDIR &fdir);
