#pragma once
#include <vgl/vgl.h>

// World coordinate-axes gizmo: three world-aligned (not body-aligned)
// arrows anchored to a fixed offset from the *camera*, not the scene, so
// they sit in a fixed screen corner and visibly rotate as the user orbits
// the camera -- the same idea as the axis gizmo in Blender/Unity/
// constellation-sim.
//
// Drawn in **screen space** via ImGui's foreground draw list -- not as 3D
// world geometry. An earlier version of this drew three arrows anchored to
// a fixed world-space offset from the camera, sized to look constant
// regardless of zoom; that technique is fundamentally fighting the 3D
// pipeline (depth testing against a scene spanning 0.1m to 1e8m, the
// logarithmic depth buffer's own precision characteristics at close range,
// perspective distortion changing the arrows' apparent proportions with
// FOV/aspect) to fake something that isn't really a 3D object at all -- a
// fixed-screen-position orientation indicator. constellation-sim's
// SatelliteRenderer::drawAxesOverlay() does this the direct way: skip 3D
// rendering for the gizmo entirely, project each world axis (a unit
// vector) onto the screen using simple dot products against the camera's
// own right/up basis vectors (an orthographic-style projection -- exactly
// right here, since the gizmo is meant to show *orientation only*, not
// position or perspective), and draw the result as 2D lines/triangles
// directly. No depth testing, no clip planes, no scale/distance math to
// get right -- ported here with this project's own axis color convention.
void drawWorldAxesGizmo(GUI &gui);
