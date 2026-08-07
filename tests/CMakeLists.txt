# Each test file is its own executable/ctest entry (matching how real
# flight-software test suites are usually organized: one binary per
# module/feature, not one monolith) -- see test_common.h for why there's no
# GoogleTest/Catch2 dependency, and docs/TESTING.md for the convention new
# tests should follow.

add_executable(test_adcs_control test_adcs_control.cpp)
target_link_libraries(test_adcs_control PRIVATE fsw)
add_test(NAME adcs_control COMMAND test_adcs_control)

add_executable(test_fdir test_fdir.cpp)
target_link_libraries(test_fdir PRIVATE fsw)
add_test(NAME fdir COMMAND test_fdir)

add_executable(test_flight_software test_flight_software.cpp)
target_link_libraries(test_flight_software PRIVATE fsw)
add_test(NAME flight_software COMMAND test_flight_software)

# Needs `rigidbody` too (not just `fsw`) -- SolarPanel::sample() takes a
# real RigidBody, since it's a physics primitive living in the engine, not
# FSW.
add_executable(test_eps test_eps.cpp)
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
