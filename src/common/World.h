#pragma once
#include <vgl/vgl.h>
#include <vector>
#include <random>

// Shared "environment" dressing for example scenarios: background color,
// lighting direction, an optional ground plane, and an optional starfield.
// Each example picks the WorldType matching its setting, calls
// world.apply(gui) once right after constructing the GUI, then world.draw(gui)
// once per frame before drawing scene content. A reference grid is NOT part
// of this -- it used to be a single infinite-ground-style plane here, but
// that's scene dressing scaled for a scenario with an actual ground; a
// scenario that wants a coordinate grid at its own object's scale (see
// satellite_adcs_sim.cpp's drawSatelliteCoordinateBox()) draws its own.
//
// This is example-level scene dressing, not a rendering primitive, so it
// lives here rather than in VGL -- it encodes conventions specific to how
// these scenarios want to look (Z-up, this project's color choices), not
// something a generic renderer should know about.
enum class WorldType
{
  SPACE,   // deep space: starfield, no ground, no atmosphere
  EARTH,   // blue sky, infinite green ground plane
  DEFAULT, // neutral dark backdrop, faint reference grid, no ground/sky/stars
};

class World
{
public:
  explicit World(WorldType type) : m_type(type)
  {
    if (m_type == WorldType::SPACE)
      m_stars = generateStars(260, kStarShellRadius);
  }

  // Call once, right after constructing the GUI.
  void apply(GUI &gui) const
  {
    switch (m_type)
    {
    case WorldType::SPACE:
      gui.setClearColor({0.015f, 0.018f, 0.035f});
      gui.setLightDirection({0.5f, 1.0f, 0.3f});
      break;
    case WorldType::EARTH:
      gui.setClearColor(kSkyColor);
      gui.setLightDirection({0.35f, 0.55f, 1.0f});
      break;
    case WorldType::DEFAULT:
    default:
      gui.setClearColor({0.09f, 0.09f, 0.1f});
      gui.setLightDirection({0.5f, 1.0f, 0.3f});
      break;
    }
  }

  // Call once per frame, before drawing scene content.
  void draw(GUI &gui) const
  {
    if (m_type == WorldType::SPACE)
      drawStars(gui);

    if (m_type == WorldType::EARTH)
      drawGround(gui);
  }

private:
  struct Star
  {
    glm::vec3 pos;
    float brightness;
  };

  WorldType m_type;
  std::vector<Star> m_stars;

  static constexpr float kStarShellRadius = 20.0f;
  static constexpr float kGroundMaxDistance = 200000.0f; // fade-out distance, not a mesh size -- see drawGround()
  static inline const glm::vec3 kSkyColor{0.45f, 0.68f, 0.92f};

  static std::vector<Star> generateStars(int count, float shellRadius)
  {
    std::vector<Star> stars;
    stars.reserve(count);

    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> dirDist(-1.0f, 1.0f);
    std::uniform_real_distribution<float> brightDist(0.35f, 1.0f);

    for (int i = 0; i < count; i++)
    {
      glm::vec3 dir;
      do
      {
        dir = glm::vec3(dirDist(rng), dirDist(rng), dirDist(rng));
      } while (glm::length(dir) < 1e-4f);
      dir = glm::normalize(dir);
      stars.push_back({dir * shellRadius, brightDist(rng)});
    }
    return stars;
  }

  void drawStars(GUI &gui) const
  {
    const glm::vec3 baseColor{0.75f, 0.8f, 0.9f};
    const float radius = kStarShellRadius * 0.0015f;
    for (const auto &star : m_stars)
      gui.drawSphere(star.pos, radius, baseColor * star.brightness);
  }

  // A true infinite plane (ray-cast per pixel in the fragment shader, see
  // EmbeddedShaders::groundPlaneFrag), not a giant mesh -- a literal huge
  // box has vertices far enough from the origin that both world-space
  // float precision and depth-buffer precision break down at distance,
  // which is exactly the "pixelated ground, vibrating rocket" artifact
  // this replaces.
  void drawGround(GUI &gui) const
  {
    const glm::vec3 grassColor{0.16f, 0.42f, 0.16f};
    gui.drawInfiniteGroundPlane(grassColor, kSkyColor, 0.0f, kGroundMaxDistance);
  }

};
