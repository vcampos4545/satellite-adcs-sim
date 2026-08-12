# Each test file is its own executable/ctest entry (matching how real
# flight-software test suites are usually organized: one binary per
# module/feature, not one monolith) -- see test_common.h for why there's no
# GoogleTest/Catch2 dependency, and docs/TESTING.md for the convention new
# tests should follow.

# ADCS is fully hardware-abstracted (see ADCS.h's own header comment) --
# these three tests drive it directly with synthetic per-sensor readings,
# so they need nothing beyond `fsw` itself (no rigidbody, no Satellite).
add_executable(test_adcs_control test_adcs_control.cpp)
target_link_libraries(test_adcs_control PRIVATE fsw)
add_test(NAME adcs_control COMMAND test_adcs_control)

add_executable(test_flight_software test_flight_software.cpp)
target_link_libraries(test_flight_software PRIVATE fsw)
add_test(NAME flight_software COMMAND test_flight_software)

add_executable(test_detumble test_detumble.cpp)
target_link_libraries(test_detumble PRIVATE fsw)
add_test(NAME detumble COMMAND test_detumble)

# FlightSoftware::step() samples sensors/applies actuator commands against
# a real Satellite (see FlightSoftware.h's own header comment) -- so this
# test needs Satellite.cpp/GroundStations.cpp compiled in and `rigidbody`
# linked, the same way test_ground_stations/test_solar_farms already pull
# in the src/core/*.cpp file their own test needs. ../src (not just
# ../src/core) is on the include path since FlightSoftware.cpp's own
# #include "core/Satellite.h" (compiled into fsw) and this test's
# #include "core/Satellite.h" both need it resolvable.
add_executable(test_fdir test_fdir.cpp ../src/core/Satellite.cpp ../src/core/GroundStations.cpp)
target_include_directories(test_fdir PRIVATE ../src)
target_link_libraries(test_fdir PRIVATE fsw rigidbody)
add_test(NAME fdir COMMAND test_fdir)

# Needs `rigidbody` too (not just `fsw`) -- SolarPanel::sample() takes a
# real RigidBody, since it's a physics primitive living in the engine, not
# FSW. Also needs Satellite.cpp/GroundStations.cpp compiled in for its
# FlightSoftware::step()-driven FDIR/battery scenarios -- see test_fdir.cpp's
# own header comment.
add_executable(test_eps test_eps.cpp ../src/core/Satellite.cpp ../src/core/GroundStations.cpp)
target_include_directories(test_eps PRIVATE ../src)
target_link_libraries(test_eps PRIVATE fsw rigidbody)
add_test(NAME eps COMMAND test_eps)

# GroundStations.cpp lives in the main src tree (src/core/), not fsw --
# it's harness-level data/logic (see its own header comment on why: no
# real need for a data-file loader over a handful of entries), so it's
# compiled directly into this test rather than linked from a library
# target, the same way any other src/core/*.cpp file would be if it ever
# needed its own persistent test.
add_executable(test_ground_stations test_ground_stations.cpp ../src/core/GroundStations.cpp)
target_include_directories(test_ground_stations PRIVATE ../src/core)
# fsw is linked only for test_common.h's own "FlightTypes.h" include (its
# shared CHECK() macro lives alongside the FSW tests above) -- this test
# doesn't otherwise touch FSW at all.
target_link_libraries(test_ground_stations PRIVATE rigidbody fsw)
add_test(NAME ground_stations COMMAND test_ground_stations)

# Same reasoning as test_ground_stations above -- SolarFarms.cpp lives in
# src/core/, compiled directly into this test rather than linked from a
# library target.
add_executable(test_solar_farms test_solar_farms.cpp ../src/core/SolarFarms.cpp)
target_include_directories(test_solar_farms PRIVATE ../src/core)
target_link_libraries(test_solar_farms PRIVATE rigidbody fsw)
add_test(NAME solar_farms COMMAND test_solar_farms)
