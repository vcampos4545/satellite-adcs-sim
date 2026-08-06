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

# Needs `rigidbody` too (not just `fsw`) -- SolarPanel::sample() takes a
# real RigidBody, since it's a physics primitive living in the engine, not
# FSW.
add_executable(test_eps test_eps.cpp)
target_link_libraries(test_eps PRIVATE fsw rigidbody)
add_test(NAME eps COMMAND test_eps)
