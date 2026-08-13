#include "WorldAxesGizmo.h"
#include "core/VisualizationConfig.h"
#include <imgui.h>
#include <cmath>

void drawWorldAxesGizmo(GUI &gui)
{
  const ImGuiIO &io = ImGui::GetIO();
  ImDrawList *dl = ImGui::GetForegroundDrawList();

  const float margin = VisualizationConfig::AXES_GIZMO_MARGIN_PX;
  const float arm = VisualizationConfig::AXES_GIZMO_ARM_PX;
  const ImVec2 origin(margin, io.DisplaySize.y - margin); // bottom-left corner

  dl->AddCircleFilled(origin, arm * 0.65f, IM_COL32(0, 0, 0, 120), 32);

  // camera right -> screen +X, camera up -> screen -Y (screen Y grows
  // downward) -- dot(axis, basis) is exactly the orthographic projection
  // of a unit axis onto that screen direction, foreshortened correctly
  // as the camera rotates since it's already just the cosine of the
  // angle between them.
  glm::vec3 camRight = gui.camera.getRight();
  glm::vec3 camUp = gui.camera.getUpVector();
  auto project = [&](glm::vec3 axisDir) -> ImVec2
  {
    float sx = glm::dot(camRight, axisDir) * arm;
    float sy = -glm::dot(camUp, axisDir) * arm;
    return ImVec2(origin.x + sx, origin.y + sy);
  };

  // Same X=red/Y=green/Z=blue convention already used for the satellite's
  // own body-axis arrows (drawSatelliteWireframe) and the coordinate grid
  // (drawGridPlane) -- this reads as "the same axis colors, just showing
  // world orientation instead of body orientation."
  struct Axis
  {
    glm::vec3 dir;
    ImU32 color;
  };
  const Axis axes[3] = {
      {{1.0f, 0.0f, 0.0f}, IM_COL32(217, 77, 82, 230)},
      {{0.0f, 1.0f, 0.0f}, IM_COL32(82, 173, 102, 230)},
      {{0.0f, 0.0f, 1.0f}, IM_COL32(82, 122, 217, 230)},
  };

  for (const Axis &axis : axes)
  {
    ImVec2 tip = project(axis.dir);
    dl->AddLine(origin, tip, axis.color, 2.0f);

    float dx = tip.x - origin.x, dy = tip.y - origin.y;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1.0f)
      continue; // axis nearly edge-on to the screen -- no visible arrowhead to draw
    float ux = dx / len, uy = dy / len;
    float px = -uy * 4.0f, py = ux * 4.0f;
    dl->AddTriangleFilled(
        ImVec2(tip.x, tip.y),
        ImVec2(tip.x - ux * 9.0f + px, tip.y - uy * 9.0f + py),
        ImVec2(tip.x - ux * 9.0f - px, tip.y - uy * 9.0f - py),
        axis.color);
  }
}
