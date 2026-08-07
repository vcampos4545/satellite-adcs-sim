#include "MagneticFieldRenderer.h"
#include "core/Config.h"
#include <rigidbody/orbit/OrbitFrames.h>
#include <glm/gtc/constants.hpp>
#include <cmath>

void drawMagneticField(GUI &gui, const glm::vec3 &fieldWorldT, glm::vec3 satPos)
{
  // Scales a ~20-60 uT LEO field into a visible arrow length, then applies
  // SATELLITE_VISUAL_SCALE like every other satellite-local dimension so
  // the field arrow stays proportionate to the now-much-bigger satellite
  // instead of shrinking to nothing next to it.
  constexpr float FIELD_VISUAL_SCALE = 8000.0f;
  const glm::vec3 fieldColor{0.2f, 0.9f, 0.9f};

  glm::vec3 arrow = fieldWorldT * FIELD_VISUAL_SCALE * Config::SATELLITE_VISUAL_SCALE;
  gui.drawArrow(satPos, satPos + arrow, fieldColor, 2.0f);
}

std::vector<FieldLine> traceDipoleFieldLines(const CentralBodyMagneticField &magField)
{
  const float earthR = static_cast<float>(OrbitFrames::EARTH_RADIUS_M);
  const float stepM = earthR * Config::FIELD_LINE_STEP_FRAC_EARTH_RADIUS;
  const float maxRadius = earthR * Config::FIELD_LINE_MAX_RADIUS_FRAC_EARTH_RADIUS;
  const glm::vec3 axis = glm::normalize(magField.rotationAxisWorld);
  glm::vec3 axisX = (std::abs(axis.x) < 0.9f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
  glm::vec3 basisX = glm::normalize(glm::cross(axisX, axis));
  glm::vec3 basisY = glm::cross(axis, basisX);

  std::vector<FieldLine> lines;
  for (bool north : {true, false})
  {
    glm::vec3 pole = north ? axis : -axis;
    for (float colatDeg : Config::FIELD_LINE_COLATITUDES_DEG)
    {
      float colat = glm::radians(colatDeg);
      for (int ai = 0; ai < Config::FIELD_LINE_AZIMUTH_COUNT; ai++)
      {
        float az = (2.0f * glm::pi<float>() * ai) / Config::FIELD_LINE_AZIMUTH_COUNT;
        glm::vec3 seedDir = std::cos(colat) * pole + std::sin(colat) * (std::cos(az) * basisX + std::sin(az) * basisY);
        glm::vec3 pos = seedDir * (earthR * 1.001f); // just above the surface, avoiding the r<1m undefined-direction case

        glm::vec3 b0 = magField.sample(pos);
        if (glm::length(b0) < 1e-30f)
          continue;
        // Integrate away from the surface first -- the dipole loop's own
        // curvature brings it back down near the other pole.
        float sign = (glm::dot(b0, glm::normalize(pos)) > 0.0f) ? 1.0f : -1.0f;

        FieldLine line;
        line.seededNorth = north;
        line.points.reserve(Config::FIELD_LINE_MAX_POINTS);
        line.points.push_back(pos);

        auto direction = [&](const glm::vec3 &p) { return sign * glm::normalize(magField.sample(p)); };
        for (int i = 0; i < Config::FIELD_LINE_MAX_POINTS; i++)
        {
          glm::vec3 k1 = direction(pos);
          glm::vec3 k2 = direction(pos + 0.5f * stepM * k1);
          glm::vec3 k3 = direction(pos + 0.5f * stepM * k2);
          glm::vec3 k4 = direction(pos + stepM * k3);
          pos += (stepM / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
          line.points.push_back(pos);

          float r = glm::length(pos);
          if (r < earthR * 1.001f && i > 2)
            break; // closed the loop back onto the surface
          if (r > maxRadius)
            break; // safety cutoff -- shouldn't trigger for these colatitudes, but near-axis seeds could
        }
        lines.push_back(std::move(line));
      }
    }
  }
  return lines;
}

void drawMagneticFieldLines(GUI &gui, const std::vector<FieldLine> &lines)
{
  const glm::vec3 northColor{0.25f, 0.65f, 0.95f};
  const glm::vec3 southColor{0.95f, 0.65f, 0.15f};
  for (const FieldLine &line : lines)
  {
    const glm::vec3 &color = line.seededNorth ? northColor : southColor;
    for (size_t i = 0; i + 1 < line.points.size(); i++)
      gui.drawLine(line.points[i], line.points[i + 1], color, 1.0f);
  }
}
