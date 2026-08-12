cmake_minimum_required(VERSION 3.16)
project(main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glm REQUIRED)
find_package(glfw3 REQUIRED)

include(FetchContent)

# rigidbody -- the generic rigid-body physics engine (bodies, actuators,
# sensors, constraints) this project's flight software runs against.
# Fetched from GitHub, not a local sibling checkout, so the build is
# reproducible for anyone cloning this repo. RIGIDBODY_BUILD_EXAMPLES
# defaults off when consumed this way (that repo only turns it on when
# it's the top-level project), so this doesn't drag in its own VGL-based
# example scenarios -- this repo owns its own flight software instead.
FetchContent_Declare(
  rigidbody
  GIT_REPOSITORY https://github.com/vcampos4545/spacecraft-dynamics-sim.git
  GIT_TAG main
)
FetchContent_MakeAvailable(rigidbody)

# VGL -- rendering.
FetchContent_Declare(
  vgl
  GIT_REPOSITORY https://github.com/vcampos4545/VGL.git
  GIT_TAG main
)
FetchContent_MakeAvailable(vgl)

# Dear ImGui -- telemetry/control panels. Fetched/built here directly
# (not by VGL, which stays generic and has no ImGui dependency) -- see
# src/core/ImGuiLayer.h for the RAII wrapper that brackets VGL's own
# frame calls with it.
#
# find_package(glfw3) was already called above, but FetchContent_
# MakeAvailable(vgl) finds its own copy of glfw inside VGL's own (child)
# CMake scope, which isn't visible back up here -- hence the explicit call
# at the top of this file rather than relying on VGL having already found
# it.
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.7
)
FetchContent_MakeAvailable(imgui)

add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw)

# ---------------------------------------------------------------------------
# fsw -- the flight software itself: PID/LQR/cascaded attitude control, a
# multiplicative EKF fed by a star tracker (with a sun+magnetometer TRIAD
# fallback when it's blinded/slewing too fast), B-dot detumble, cross-
# product-law desaturation, and the FDIR mode-manager/fault layer above all
# of it. Split into its own library (not just object files baked into the
# main executable) so tests/ can link exactly this and nothing else --
# ADCS.cpp/Controllers.cpp/FDIR.cpp are fully hardware-abstracted (see
# ADCS.h's own header comment) and need only glm, proven by construction:
# test_adcs_control/test_detumble/test_flight_software link `fsw` alone.
#
# FlightSoftware.cpp is the one file here that isn't hardware-abstracted --
# it's this project's bridge to a real Satellite (src/core/Satellite.h),
# sampling sensors/commanding actuators directly (see its own header
# comment on why that's not a Satellite method) -- so it needs `rigidbody`
# and src/core's headers (Satellite.h/GroundStations.h/Config.h) too. Any
# test/ target that actually drives FlightSoftware (not just ADCS) needs to
# compile src/core/Satellite.cpp (+ GroundStations.cpp) and link rigidbody
# in turn -- see tests/CMakeLists.txt. Still never needs vgl/imgui: nothing
# here renders or plots anything -- telemetry-for-the-UI stays the
# harness's (main.cpp's) job.
# ---------------------------------------------------------------------------
add_library(fsw STATIC
  src/fsw/ADCS.cpp
  src/fsw/Controllers.cpp
  src/fsw/FDIR.cpp
  src/fsw/FlightSoftware.cpp
)
# PUBLIC include dir is src/fsw itself (not all of src) -- fsw's public
# interface is only its own headers: a consumer of libfsw (tests/, main)
# sees ADCS.h/FDIR.h/Controllers.h/FlightTypes.h/FlightSoftware.h as
# unprefixed includes, not the rest of this project's source tree. The
# `src` root is PRIVATE (only FlightSoftware.cpp uses it, to reach
# src/core/*.h -- e.g. its own #include "core/Satellite.h") since the
# public headers only forward-declare Satellite/GroundStation, never
# including their full definitions themselves.
target_include_directories(fsw PUBLIC src/fsw)
target_include_directories(fsw PRIVATE src)
target_link_libraries(fsw PUBLIC glm::glm)
target_link_libraries(fsw PRIVATE rigidbody)

# ---------------------------------------------------------------------------
# The satellite: builds the simulated hardware/sensors and the scene/UI
# around them; FlightSoftware (src/fsw/FlightSoftware.h) drives them each
# cycle -- see README.md for the full feature list and this project's scope
# relative to spacecraft-dynamics-sim (the engine) and constellation-sim
# (orbit-level mission design).
# ---------------------------------------------------------------------------
add_executable(main
  src/core/main.cpp
  src/core/Satellite.cpp
  src/core/Simulation.cpp
  src/core/GroundStations.cpp
  src/core/SolarFarms.cpp
  src/rendering/SatelliteRenderer.cpp
  src/rendering/MagneticFieldRenderer.cpp
  src/rendering/OrbitRenderer.cpp
  src/rendering/WorldAxesGizmo.cpp
  src/panels/FswPanel.cpp
  src/panels/SensorsPanel.cpp
  src/panels/ActuatorsPanel.cpp
  src/panels/FdirPanel.cpp
  src/panels/EpsPanel.cpp
  src/panels/OrbitPanel.cpp
  src/panels/GroundStationsPanel.cpp
  src/panels/SimulationPanel.cpp
  src/panels/VisualizationPanel.cpp
  src/panels/ADCSPanel.cpp
  src/panels/FpsOverlay.cpp
)
# PRIVATE src (the root, not a specific subfolder) -- executable-only code
# reaches across the core/fsw/rendering/panels split via folder-qualified
# includes (e.g. "fsw/ADCS.h", "rendering/OrbitRenderer.h"), unlike fsw's
# own narrower PUBLIC include dir above.
target_include_directories(main PRIVATE src)
target_link_libraries(main PRIVATE fsw rigidbody vgl imgui)

# Absolute path to resources/ (textures, ...) baked in at compile time --
# this is a dev sim run straight from the build tree, not an installed/
# packaged product, so an absolute source-tree path is simpler than a
# POST_BUILD copy-next-to-the-executable step and works regardless of the
# working directory the built binary is launched from.
target_compile_definitions(main PRIVATE
  RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

# ---------------------------------------------------------------------------
# Tests -- see tests/test_common.h and docs/TESTING.md.
# ---------------------------------------------------------------------------
enable_testing()
add_subdirectory(tests)
