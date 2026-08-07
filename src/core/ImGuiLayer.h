#pragma once
#include <vgl/vgl.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// Thin RAII wrapper around Dear ImGui's GLFW+OpenGL3 backend, deliberately
// kept out of VGL entirely -- VGL is a generic rendering library and has no
// ImGui dependency at all, so any scenario that wants telemetry panels owns
// its own ImGuiLayer instance instead. Bracket VGL's own frame calls with
// it:
//
//   GUI gui(...);
//   ImGuiLayer imguiLayer(gui);
//   while (!gui.shouldClose()) {
//     ...
//     gui.beginFrame();
//     imguiLayer.beginFrame();
//     // ... draw 3D scene, then ImGui::Begin/.../End ...
//     imguiLayer.endFrame();
//     gui.endFrame();
//   }
//
// Fetched/compiled by this project's own CMakeLists.txt (a standalone
// `imgui` target), not by VGL's.
class ImGuiLayer
{
public:
  explicit ImGuiLayer(GUI &gui)
  {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    // true: chains onto whatever GLFW callbacks VGL's GUI already installed
    // (GUI::setupCallbacks(), called from the GUI constructor before this
    // runs), so VGL's own input handling keeps working unchanged.
    ImGui_ImplGlfw_InitForOpenGL(gui.getWindow(), true);
    ImGui_ImplOpenGL3_Init("#version 150");
  }

  ~ImGuiLayer()
  {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
  }

  ImGuiLayer(const ImGuiLayer &) = delete;
  ImGuiLayer &operator=(const ImGuiLayer &) = delete;

  // Call once per frame, right after gui.beginFrame() -- opens the ImGui
  // frame so scenario code can call ImGui::Begin/End anywhere afterward.
  void beginFrame() const
  {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
  }

  // Call once per frame, right before gui.endFrame() -- draws whatever
  // ImGui panels were built this frame on top of the already-drawn 3D scene.
  void endFrame() const
  {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  }
};
