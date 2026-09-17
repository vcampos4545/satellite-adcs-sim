#pragma once
#include <vgl/vgl.h>
#include <rigidbody/orbit/OrbitState.h>
#include <vector>

// Orbit tab: real orbital elements (from the true propagated state, not
// the commanded initial condition -- these drift from the initial
// circular/51.6deg values as J2 acts on the orbit) plus the ground track.
void drawOrbitTab(const OrbitState &orbitState, const Texture &earthTexture,
                  const std::vector<glm::vec2> &groundTrack, double currentJd);
