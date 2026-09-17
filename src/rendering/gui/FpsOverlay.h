#pragma once

// Small always-on-top overlay pinned to the screen's top-right corner,
// independent of the ADCS panel/tabs -- just Dear ImGui's own smoothed
// io.Framerate, not something this project computes itself.
void drawFpsOverlay();
