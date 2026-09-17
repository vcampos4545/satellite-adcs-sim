#include "FpsOverlay.h"
#include <imgui.h>

void drawFpsOverlay()
{
  const float pad = 10.0f;
  ImGuiIO &io = ImGui::GetIO();
  ImVec2 windowPos(io.DisplaySize.x - pad, pad);
  ImVec2 pivot(1.0f, 0.0f); // anchor the window's top-right corner to windowPos, not its top-left
  ImGui::SetNextWindowPos(windowPos, ImGuiCond_Always, pivot);
  ImGui::SetNextWindowBgAlpha(0.35f);

  // NoInputs -- this sits on top of the 3D view; it must never intercept a
  // click/drag the camera would otherwise get.
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                           ImGuiWindowFlags_NoNav | ImGuiWindowFlags_AlwaysAutoResize |
                           ImGuiWindowFlags_NoInputs;
  if (ImGui::Begin("FPS Overlay", nullptr, flags))
    ImGui::Text("%.1f FPS (%.2f ms)", io.Framerate, 1000.0f / io.Framerate);
  ImGui::End();
}
