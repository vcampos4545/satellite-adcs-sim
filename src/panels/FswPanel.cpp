#include "FswPanel.h"
#include <imgui.h>

const char *modeName(PointingMode m)
{
  switch (m)
  {
  case PointingMode::NADIR:
    return "NADIR";
  case PointingMode::SUN_POINTING:
    return "SUN_POINTING";
  case PointingMode::DETUMBLE:
    return "DETUMBLE";
  case PointingMode::TARGET:
    return "TARGET";
  case PointingMode::SLEW:
    return "SLEW";
  case PointingMode::FINE_POINTING:
    return "FINE_POINTING";
  case PointingMode::REFLECT:
    return "REFLECT";
  }
  return "?";
}

const char *controllerName(ControllerType c)
{
  switch (c)
  {
  case ControllerType::PID:
    return "PID";
  case ControllerType::LQR:
    return "LQR";
  case ControllerType::CASCADED:
    return "Cascaded PID";
  }
  return "?";
}

const char *detumbleActuatorName(DetumbleActuator a)
{
  switch (a)
  {
  case DetumbleActuator::REACTION_WHEELS:
    return "Reaction Wheels";
  case DetumbleActuator::MAGNETORQUERS_BDOT:
    return "Magnetorquers (B-dot)";
  }
  return "?";
}

void drawFswTab(ADCS &adcs, SensorTelemetry &telemetry, float trueErrDeg)
{
  static const char *modeNames[] = {"Nadir", "Sun Pointing", "Detumble", "Target", "Slew", "Fine Pointing", "Reflect"};
  int modeIdx = static_cast<int>(adcs.mode);
  if (ImGui::Combo("Pointing mode", &modeIdx, modeNames, IM_ARRAYSIZE(modeNames)))
    adcs.mode = static_cast<PointingMode>(modeIdx);
  ImGui::TextDisabled("This is what's commanded -- FDIR can override it (see FDIR tab).");

  if (adcs.mode != adcs.effectiveMode)
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "FDIR override active: flying %s instead", modeName(adcs.effectiveMode));

  if (adcs.manualOverride)
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "Manual actuator override is active (Actuators tab) -- FSW is not commanding hardware.");

  ImGui::SeparatorText("Pointing Error");
  if (adcs.mode == PointingMode::DETUMBLE)
    ImGui::TextDisabled("DETUMBLE has no attitude target; showing the last value from before it was entered.");
  ImGui::Text("FSW estimate: %.2f deg", adcs.estimatedPointingErrorDeg);
  plotChannel("Estimated error", telemetry.estimatedPointingErrorDeg, "deg");
  ImGui::Text("True (ground truth, diagnostic only): %.2f deg", trueErrDeg);
  plotChannel("True error", telemetry.truePointingErrorDeg, "deg");
  ImGui::Text("Estimator confidence (1-sigma): %.4f deg (%.1f arcsec)",
              adcs.attitudeUncertaintyDeg, adcs.attitudeUncertaintyDeg * 3600.0f);
  ImGui::TextDisabled("Grows during a star-tracker dropout, shrinks once a correction lands again.");

  ImGui::SeparatorText("Attitude Controller");
  if (adcs.mode == PointingMode::DETUMBLE)
    ImGui::TextDisabled("DETUMBLE ignores this and uses the Detumble Actuator law below instead.");

  static const char *controllerNames[] = {"PID", "LQR", "Cascaded PID"};
  int controllerIdx = static_cast<int>(adcs.controllerType);
  if (ImGui::Combo("Algorithm", &controllerIdx, controllerNames, IM_ARRAYSIZE(controllerNames)))
    adcs.controllerType = static_cast<ControllerType>(controllerIdx);

  switch (adcs.controllerType)
  {
  case ControllerType::PID:
  {
    PIDController &c = adcs.pidController();
    ImGui::DragFloat("Kp", &c.Kp, 0.0001f, 0.0f, 1.0f, "%.5f");
    ImGui::DragFloat("Ki", &c.Ki, 0.00001f, 0.0f, 0.1f, "%.6f");
    ImGui::DragFloat("Kd", &c.Kd, 0.0001f, 0.0f, 1.0f, "%.5f");
    ImGui::DragFloat("Max integral", &c.maxIntegral, 0.001f, 0.0f, 1.0f, "%.4f");
    break;
  }
  case ControllerType::LQR:
  {
    LQRController &c = adcs.lqrController();
    ImGui::DragFloat3("Kp (x,y,z)", &c.Kp.x, 0.0001f, 0.0f, 1.0f, "%.5f");
    ImGui::DragFloat3("Kd (x,y,z)", &c.Kd.x, 0.0001f, 0.0f, 1.0f, "%.5f");
    ImGui::DragFloat("Omega max (rad/s)", &c.omega_max, 0.01f, 0.01f, 5.0f, "%.3f");
    ImGui::TextDisabled("Q/R weights (derived by auto-tune, view only):");
    ImGui::Text("Q_att:  %.4f  %.4f  %.4f", c.Q_att.x, c.Q_att.y, c.Q_att.z);
    ImGui::Text("Q_rate: %.4f  %.4f  %.4f", c.Q_rate.x, c.Q_rate.y, c.Q_rate.z);
    ImGui::Text("R:      %.4f  %.4f  %.4f", c.R.x, c.R.y, c.R.z);
    break;
  }
  case ControllerType::CASCADED:
  {
    CascadedController &c = adcs.cascadedController();
    ImGui::DragFloat("Settling time (s)", &c.settlingTime, 0.1f, 0.5f, 30.0f, "%.2f");
    ImGui::DragFloat("Damping ratio", &c.dampingRatio, 0.01f, 0.1f, 3.0f, "%.3f");
    ImGui::DragFloat("Omega max (rad/s)", &c.omega_max, 0.01f, 0.01f, 5.0f, "%.3f");
    break;
  }
  }
  if (ImGui::Button("Reset to auto-tuned gains for current mode"))
    adcs.retuneForMode();

  ImGui::SeparatorText("Detumble Actuator");
  static const char *detumbleNames[] = {"Reaction Wheels", "Magnetorquers (B-dot)"};
  int detumbleIdx = static_cast<int>(adcs.detumbleActuator);
  if (ImGui::Combo("Detumble via", &detumbleIdx, detumbleNames, IM_ARRAYSIZE(detumbleNames)))
    adcs.detumbleActuator = static_cast<DetumbleActuator>(detumbleIdx);

  if (adcs.detumbleActuator == DetumbleActuator::MAGNETORQUERS_BDOT)
  {
    ImGui::DragFloat("B-dot gain (A*m^2 per T/s)", &adcs.bdotGain, 100.0f, 0.0f, 1.0e7f, "%.0f");
    ImGui::Text("dB/dt (body): %.2f uT/s -- see Sensors tab for the field itself", glm::length(adcs.magFieldRateBody) * 1e6f);
  }
}
