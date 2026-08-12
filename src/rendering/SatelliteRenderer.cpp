#include "SatelliteRenderer.h"
#include "core/Config.h"

void drawSatelliteWireframe(GUI &gui, RigidBody *sat)
{
  glm::vec3 h = sat->size * 0.5f * Config::SATELLITE_VISUAL_SCALE;
  glm::quat q = sat->orientation;
  glm::vec3 p = sat->position;
  const glm::vec3 color{1.0f, 1.0f, 0.0f};

  glm::vec3 c[8] = {
      p + q * glm::vec3(-h.x, -h.y, -h.z),
      p + q * glm::vec3(+h.x, -h.y, -h.z),
      p + q * glm::vec3(+h.x, +h.y, -h.z),
      p + q * glm::vec3(-h.x, +h.y, -h.z),
      p + q * glm::vec3(-h.x, -h.y, +h.z),
      p + q * glm::vec3(+h.x, -h.y, +h.z),
      p + q * glm::vec3(+h.x, +h.y, +h.z),
      p + q * glm::vec3(-h.x, +h.y, +h.z),
  };

  // Bottom face (4)
  gui.drawLine(c[0], c[1], color);
  gui.drawLine(c[1], c[2], color);
  gui.drawLine(c[2], c[3], color);
  gui.drawLine(c[3], c[0], color);
  // Top face (4)
  gui.drawLine(c[4], c[5], color);
  gui.drawLine(c[5], c[6], color);
  gui.drawLine(c[6], c[7], color);
  gui.drawLine(c[7], c[4], color);
  // Vertical edges (4)
  gui.drawLine(c[0], c[4], color);
  gui.drawLine(c[1], c[5], color);
  gui.drawLine(c[2], c[6], color);
  gui.drawLine(c[3], c[7], color);

  float arrowLength = 0.25f * Config::SATELLITE_VISUAL_SCALE;
  gui.drawArrow(p, p + q * glm::vec3(1, 0, 0) * arrowLength, glm::vec3(1, 0, 0));
  gui.drawArrow(p, p + q * glm::vec3(0, 1, 0) * arrowLength, glm::vec3(0, 1, 0));
  gui.drawArrow(p, p + q * glm::vec3(0, 0, 1) * arrowLength, glm::vec3(0, 0, 1));
}

void drawReactionWheels(GUI &gui, const std::vector<ReactionWheel *> &reactionWheels, RigidBody *sat)
{
  const float wheelRadius = 0.02f * Config::SATELLITE_VISUAL_SCALE;
  const float wheelThickness = 0.006f * Config::SATELLITE_VISUAL_SCALE;
  const float arrowLength = 0.05f * Config::SATELLITE_VISUAL_SCALE;

  glm::vec3 totalAngular{0};
  for (auto &wheel : reactionWheels)
  {
    // wheel->getWorldMountPosition(*sat) computes sat->position +
    // sat->orientation*mountPositionBody *inside the engine*, in float32,
    // before we ever see the result -- at a real orbital position
    // (~6.9e6 m), float32's ULP there (~0.8 m) is *larger* than the
    // ~0.03 m mount offset itself, so that addition rounds the offset
    // away almost entirely before SATELLITE_VISUAL_SCALE ever gets
    // applied to it (this was the wheels'/rods' actual rendering bug --
    // not a scale-factor mistake, a precision loss that happened before
    // any of this function's own math runs). Fixed by reading
    // mountPositionBody directly (a public field) and applying
    // SATELLITE_VISUAL_SCALE to the *offset* first -- turning a ~0.03 m
    // value into a ~600 m one, comfortably above that same ULP -- before
    // rotating and adding it to sat->position, exactly the order
    // drawMirror already uses for its own mount offset.
    glm::vec3 worldPos = sat->position + sat->orientation * (wheel->mountPositionBody * Config::SATELLITE_VISUAL_SCALE);
    glm::vec3 worldAxis = wheel->getWorldSpinAxis(*sat);

    glm::vec3 color;
    if (wheel->healthFactor <= 0.01f)
      color = {0.15f, 0.15f, 0.15f}; // dead
    else if (wheel->healthFactor < 0.99f)
      color = {0.85f, 0.1f, 0.85f}; // degraded
    else
    {
      float satRatio = wheel->getSaturationRatio();
      float absSatRatio = std::abs(satRatio);
      if (absSatRatio < 0.5f)
        color = {0, 1, 0};
      else if (absSatRatio < 0.9f)
        color = {1, 1, 0};
      else
        color = {1, 0, 0};
    }

    // Flat "puck": thin along the spin axis, wide across it.
    gui.drawCylinder(worldPos, wheelRadius, wheelThickness, worldAxis, glm::quat(1, 0, 0, 0), color);

    // Speed arrow from the wheel's own center, along its spin axis --
    // reflects actual current speed regardless of health.
    float satRatio = wheel->getSaturationRatio();
    gui.drawArrow(worldPos, worldPos + worldAxis * arrowLength * satRatio, color);
    totalAngular += worldAxis * satRatio;
  }
  gui.drawArrow(sat->position, sat->position + totalAngular * arrowLength * 4.0f, {1.0f, 0.65f, 0});
}

void drawMagnetorquers(GUI &gui, const std::vector<Magnetorquer *> &magnetorquers, RigidBody *sat)
{
  const float rodRadius = 0.006f * Config::SATELLITE_VISUAL_SCALE;
  const float rodLength = 0.035f * Config::SATELLITE_VISUAL_SCALE;
  const float arrowLength = 0.06f * Config::SATELLITE_VISUAL_SCALE;

  for (auto &rod : magnetorquers)
  {
    // See drawReactionWheels' equivalent comment -- same precision fix:
    // scale the local mount offset before rotating/adding it, not after.
    glm::vec3 worldPos = sat->position + sat->orientation * (rod->mountPositionBody * Config::SATELLITE_VISUAL_SCALE);
    glm::vec3 worldAxis = rod->getWorldAxis(*sat);

    float satRatio = rod->getSaturationRatio();
    float absSatRatio = std::abs(satRatio);
    glm::vec3 color;
    if (absSatRatio < 0.5f)
      color = {0.2f, 0.6f, 1.0f}; // blue-ish (idle/light use) to distinguish from wheels' green
    else if (absSatRatio < 0.9f)
      color = {1, 1, 0};
    else
      color = {1, 0, 0};

    gui.drawCylinder(worldPos, rodRadius, rodLength, worldAxis, glm::quat(1, 0, 0, 0), color);
    gui.drawArrow(worldPos, worldPos + worldAxis * arrowLength * satRatio, color);
  }
}

void drawBus(GUI &gui, RigidBody *sat)
{
  // mountOffsetBody is computed from the satellite's true (18m-scale)
  // size, then scaled -- this is a pure body-frame local offset, not yet
  // rotated into world space, so scaling it here (before applying
  // orientation) is equivalent to and simpler than the
  // offset-from-center trick drawReactionWheels/drawMagnetorquers use.
  // Along -Z (behind the mirror plate) so the bus reads as a distinct
  // structure sitting at the mirror's center, not merged into it.
  glm::vec3 mountOffsetBody = -Config::MIRROR_NORMAL_BODY * (sat->size.z * 0.5f + Config::BUS_SIZE.z * 0.5f) * Config::SATELLITE_VISUAL_SCALE;
  glm::vec3 worldPos = sat->position + sat->orientation * mountOffsetBody;
  gui.drawBox(worldPos, Config::BUS_SIZE * Config::SATELLITE_VISUAL_SCALE, sat->orientation, {0.6f, 0.62f, 0.68f});
}

void drawSunReflection(GUI &gui, RigidBody *sat, const glm::vec3 &sunPosition)
{
  // `sat` itself is the mirror plate now -- its own position/orientation
  // is the reflecting surface, no mount-offset math needed (that offset
  // only existed while the decorative mirror was physically separate
  // from the tiny cube body).
  glm::vec3 mirrorPos = sat->position;
  glm::vec3 normalWorld = glm::normalize(sat->orientation * Config::MIRROR_NORMAL_BODY);

  // sunPosition is the real Sun position (~1.5e11 m away, see main()) --
  // mirrorPos sitting at the satellite's real orbital position is
  // negligible against that distance either way.
  glm::vec3 incidentDir = glm::normalize(mirrorPos - sunPosition); // sun -> mirror
  gui.drawLine(sunPosition, mirrorPos, {1.0f, 0.5f, 0.9f});

  // Front face is illuminated only if the normal points back toward the
  // sun relative to the incoming ray, i.e. dot(normal, -incident) > 0.
  if (glm::dot(normalWorld, -incidentDir) <= 0.0f)
    return;

  glm::vec3 reflectedDir = incidentDir - 2.0f * glm::dot(incidentDir, normalWorld) * normalWorld;
  gui.drawLine(mirrorPos, mirrorPos + reflectedDir * Config::REFLECTED_RAY_LENGTH * Config::SATELLITE_VISUAL_SCALE, {1.0f, 0.5f, 0.9f});
}
