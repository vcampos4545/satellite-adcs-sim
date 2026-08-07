#pragma once
#include "core/GroundStations.h"
#include <vector>

// Ground Stations tab: the predicted contact schedule across all
// GROUND_STATIONS (see GroundStations.h's predictGroundStationPasses) as
// a sortable table -- Station / AOS / LOS / Duration / Max elevation --
// plus a detail card for whichever pass is selected, in the field-label
// layout real ground-station/mission-ops software uses. `selectedIndex`
// is held by the caller (main()) across frames the same way other
// per-frame UI selection state is; -1 means no row selected. Fields this
// project can't yet compute honestly (data rate, link margin, weather,
// scheduling priority -- no link-budget/weather/scheduling model exists)
// are shown as explicitly "not yet modeled," never a fabricated number.
void drawGroundStationsTab(const std::vector<GroundStationPass> &passes, int &selectedIndex);
