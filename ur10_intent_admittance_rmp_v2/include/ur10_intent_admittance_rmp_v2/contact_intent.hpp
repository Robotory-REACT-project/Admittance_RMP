#pragma once

// v2 goal 2 (2026-11-07): continuous intent instead of the binary contact latch.
//
// The vision (through vision_intent_bridge.py) gives lambda in [0, 1], but its
// intention disappears the moment the hand stops on the cover.  The old latch
// bridged that with a boolean state (enter: p >= 0.6 and F >= enter force,
// release: F <= release force).  Here the same job is done by a smooth
// "evidence memory" m:
//
//   hold(F)       = smoothstep((|F| - hold_low) / (hold_high - hold_low))
//   m            <- lambda_eff * exp(-(1 - hold(F)) * dt / release_tau)
//   lambda_eff    = max(lambda, m)
//
// A firm push (hold = 1) keeps the intent that was already there; without
// contact force it fades with release_tau.  Force alone can never create
// intent (m starts from lambda), so an unintended push still meets the target
// and the avoidance.  Everything downstream is a smooth function of
// lambda_eff:
//
//   compliance gate  c = smoothstep((lambda_eff - c_low) / (c_high - c_low))
//   avoidance weight w = max(floor, 1 - lambda_eff)
//
// and the target metric scale may be geometric, floor^a, so that an
// intermediate activation a gives an intermediate competition against the
// other flows (with floor + (1 - a)(1 - floor) the 1e5 target metric wins
// until a is within ~1e-3 of 1).

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ur10_intent_admittance_rmp_v2
{

inline double intent_smoothstep(double x)
{
  x = std::clamp(x, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

struct IntentEvidenceParams
{
  double hold_force_low{1.0};
  double hold_force_high{3.0};
  double release_tau{0.5};
  double compliance_low{0.5};
  double compliance_high{0.8};

  void validate() const
  {
    if (!(std::isfinite(hold_force_low) && std::isfinite(hold_force_high) &&
      hold_force_low >= 0.0 && hold_force_high > hold_force_low &&
      std::isfinite(release_tau) && release_tau > 0.0 &&
      std::isfinite(compliance_low) && std::isfinite(compliance_high) &&
      compliance_low >= 0.0 && compliance_high > compliance_low && compliance_high <= 1.0))
    {
      throw std::invalid_argument("invalid intent evidence parameters");
    }
  }
};

struct IntentEvidenceState
{
  double memory{0.0};
};

// One control cycle.  lambda outside [0, 1] or non-finite is treated as 0
// (no intent); a non-finite force as no force.
inline double update_intent_evidence(
  IntentEvidenceState & state, double lambda, double force_norm, double dt,
  const IntentEvidenceParams & params)
{
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument("invalid intent evidence time step");
  }
  const double lam = std::isfinite(lambda) ? std::clamp(lambda, 0.0, 1.0) : 0.0;
  const double force = std::isfinite(force_norm) ? std::max(force_norm, 0.0) : 0.0;
  const double hold = intent_smoothstep(
    (force - params.hold_force_low) / (params.hold_force_high - params.hold_force_low));
  const double decayed = state.memory * std::exp(-(1.0 - hold) * dt / params.release_tau);
  const double effective = std::max(lam, decayed);
  state.memory = effective;
  return effective;
}

inline double intent_compliance_gate(double lambda_eff, const IntentEvidenceParams & params)
{
  return intent_smoothstep(
    (lambda_eff - params.compliance_low) / (params.compliance_high - params.compliance_low));
}

// floor^activation (geometric) or floor + (1 - a)(1 - floor) (linear).
inline double contact_target_scale(double activation, double floor, bool geometric)
{
  const double a = std::clamp(std::isfinite(activation) ? activation : 0.0, 0.0, 1.0);
  const double f = std::clamp(floor, 0.0, 1.0);
  if (geometric && f > 0.0) {
    return std::pow(f, a);
  }
  return f + (1.0 - a) * (1.0 - f);
}

}  // namespace ur10_intent_admittance_rmp_v2
