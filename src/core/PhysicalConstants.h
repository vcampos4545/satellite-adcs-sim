#pragma once

// Universal physical/astronomical constants -- independent of which
// satellite is being simulated or how anything is drawn. See
// SatelliteConfig.h for this project's specific spacecraft design/FSW
// timing, and VisualizationConfig.h for rendering/camera/UI knobs.
namespace PhysicalConstants
{
  // Solar constant at 1 AU (W/m^2) -- SolarPanel generation (buildSatellite's
  // EPS model, EpsPanel's live readout) both scale off this real-world
  // figure, not something satellite- or scene-specific.
  constexpr float SOLAR_FLUX_WM2 = 1361.0f;

  // Standard gravitational parameters (m^3/s^2) for the Sun/Moon third-body
  // perturbations on the orbit -- see docs/ALGORITHMS.md's "Orbital
  // Mechanics" section. Earth's own GM/radius live in the
  // spacecraft-dynamics-sim engine (TwoBodyGravity/OrbitFrames), not here,
  // since this project never varies them.
  constexpr double GM_SUN = 1.32712440018e20;
  constexpr double GM_MOON = 4.9048695e12;
}
