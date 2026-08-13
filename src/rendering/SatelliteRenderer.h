#pragma once
#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Magnetorquer.h>
#include <vector>

// ---------------------------------------------------------------------------
// Satellite-local 3D draw helpers: the body wireframe (the real 18m x 18m
// mirror plate), reaction wheels, magnetorquers, the (non-physical,
// visualization-only) bus box, and the sun-reflection geometry --
// everything drawn relative to the satellite's own body frame and scaled
// by VisualizationConfig::SATELLITE_VISUAL_SCALE. See MagneticFieldRenderer.h/
// OrbitRenderer.h for the global (Earth-scale) visualizations.
// ---------------------------------------------------------------------------

// Body drawn as a 12-edge wireframe box instead of a solid box.
void drawSatelliteWireframe(GUI &gui, RigidBody *sat);

// Wheels drawn as flat cylinders at their actual mount position/orientation,
// with a speed arrow from each wheel's center. Color communicates health
// first (magenta = degraded, near-black = dead) and saturation only for
// wheels that are actually healthy.
void drawReactionWheels(GUI &gui, const std::vector<ReactionWheel *> &reactionWheels, RigidBody *sat);

// Torque rods drawn as thin cylinders along their mounted axis (unlike the
// wheels' flat pucks -- a physical torque rod is a long, thin coil, not a
// disc). Color scales with saturation (how close to max commanded moment),
// same 3-stop green/yellow/red convention drawReactionWheels uses; an
// arrow from the rod's center along its axis shows the sign/magnitude of
// the currently commanded dipole moment.
void drawMagnetorquers(GUI &gui, const std::vector<Magnetorquer *> &magnetorquers, RigidBody *sat);

// A small decorative box mounted just behind the body's -Z face,
// representing the bus core at the center of the real 18m x 18m mirror
// plate `sat` itself now is -- see VisualizationConfig::BUS_SIZE. Not a physics body,
// purely visual (the composite mirror+bus mass/inertia is already baked
// into `sat` by buildSatellitePyramid()).
void drawBus(GUI &gui, RigidBody *sat);

// Draws the incoming ray from the sun to the mirror, and the outgoing
// (reflected) ray away from it, via the ordinary law of reflection
// (angle of incidence = angle of reflection about the mirror's normal).
// The reflected ray is only drawn when the mirror's front face is
// actually sun-facing -- reflecting a ray that's hitting the mirror's
// back would be nonsense, not just an unlikely case a real mirror can't
// do either. `sat` itself is the mirror (its +Z face, MIRROR_NORMAL_BODY),
// so the reflection geometry originates directly from its position/
// orientation -- no separate mount offset needed.
void drawSunReflection(GUI &gui, RigidBody *sat, const glm::vec3 &sunPosition);
