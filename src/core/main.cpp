#include <vgl/vgl.h>
#include "ImGuiLayer.h"
#include "Config.h"
#include "SimulationState.h"
#include "Fsw.h"
#include <cstdio>

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(800, 600, "Satellite Simulator");
  ImGuiLayer imguiLayer(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);
  // The scene spans a 0.1m cubesat up to a ~6.9e6m orbital radius -- a
  // standard depth buffer can't hold that range without z-fighting.
  gui.setLogDepth(Config::CAMERA_FAR);
  gui.setAmbientLight(Config::SCENE_AMBIENT_LIGHT);

  SimulationState sim = buildSimulationState();
  Fsw fsw(sim);

  glm::vec2 lastMousePos = gui.getMousePosition();
  int selectedPassIndex = -1; // Ground Stations tab's selected table row -- transient UI state, not simulation state
  float fswTimer = 0.0f;
  float lastTime = glfwGetTime();

  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;

    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;
    glm::vec2 scrollDelta = gui.getScrollDelta();

    // One fixed-rate loop drives sim.step() (orbit propagation +
    // PhysicsWorld::step(), zero-order hold against the previous cycle's
    // actuator commands) and one fsw.step() cycle together. A `while`,
    // not an `if`: a slow render frame can span more than one nominal
    // Config::TIME_STEP_S cycle -- an `if` would silently run everything
    // at a slower relative rate, degrading pointing/detumble stability
    // for no physical reason.
    fswTimer += dt;
    while (fswTimer > Config::TIME_STEP_S)
    {
      fswTimer -= Config::TIME_STEP_S;
      sim.step(Config::TIME_STEP_S);
      fsw.step(Config::TIME_STEP_S);
    }

    sim.handleCameraInput(gui, mouseDelta, scrollDelta);

    sim.refreshGroundStationPasses(dt);

    gui.beginFrame();
    imguiLayer.beginFrame();
    sim.draw(gui, fsw, selectedPassIndex);
    imguiLayer.endFrame();
    gui.endFrame();
  }
  return 0;
}
