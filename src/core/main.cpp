#include <vgl/vgl.h>
#include "ImGuiLayer.h"
#include "VisualizationConfig.h"
#include "SatelliteConfig.h"
#include "Simulation.h"
#include "fsw/FlightSoftware.h"
#include <cstdio>

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(800, 600, "Satellite Simulator");
  glfwSwapInterval(1); // vsync on by default; toggled live via sim.simControls.vsyncEnabled (Simulation tab)
  ImGuiLayer imguiLayer(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(VisualizationConfig::CAMERA_NEAR, VisualizationConfig::CAMERA_FAR)
      .setFOV(VisualizationConfig::CAMERA_FOV);
  // The scene spans a 0.1m Satellite up to a ~6.9e6m orbital radius -- a
  // standard depth buffer can't hold that range without z-fighting.
  gui.setLogDepth(VisualizationConfig::CAMERA_FAR);
  gui.setAmbientLight(VisualizationConfig::SCENE_AMBIENT_LIGHT);

  Simulation sim = buildSimulation();
  FlightSoftware fsw(sim.spacecraft);
  fsw.configure(sim.spacecraft.hwConfig, sim.spacecraft.body->orientation);

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

    // Update
    fswTimer += dt * sim.simControls.timeScale;
    // Caps how much backlog one frame will catch up on -- see its own
    // comment in SatelliteConfig.h. Without this, a high timeScale (or a
    // real stall) would demand hundreds of catch-up steps in a single
    // frame and freeze rendering until it worked through them.
    if (fswTimer > SatelliteConfig::FSW_TIMER_MAX_S)
      fswTimer = SatelliteConfig::FSW_TIMER_MAX_S;
    while (fswTimer > SatelliteConfig::TIME_STEP_S)
    {
      fswTimer -= SatelliteConfig::TIME_STEP_S;
      sim.step(SatelliteConfig::TIME_STEP_S);
      fsw.step(SatelliteConfig::TIME_STEP_S, sim.world.orbitalState(sim.spacecraft.body).position,
               sim.currentJdNow, sim.sunPositionNow, sim.fieldNow, sim.inEclipse);
      sim.updateTelemetry(fsw);
    }

    sim.handleCameraInput(gui, mouseDelta, scrollDelta);
    sim.refreshGroundStationPasses(dt);

    // Live-toggleable (Simulation tab) -- cheap to call every frame; only
    // actually changes driver state when the checkbox flips.
    glfwSwapInterval(sim.simControls.vsyncEnabled ? 1 : 0);

    // Draw
    gui.beginFrame();
    imguiLayer.beginFrame();
    sim.draw(gui, fsw, selectedPassIndex);
    imguiLayer.endFrame();
    gui.endFrame();
  }
  return 0;
}
