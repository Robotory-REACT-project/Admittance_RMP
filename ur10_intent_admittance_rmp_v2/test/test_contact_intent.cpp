#include <cmath>

#include <gtest/gtest.h>

#include "ur10_intent_admittance_rmp_v2/contact_intent.hpp"

namespace ur10_intent_admittance_rmp_v2
{

namespace
{
constexpr double kDt = 1.0 / 125.0;

double run(IntentEvidenceState & s, double lambda, double force, double seconds,
  const IntentEvidenceParams & p)
{
  double out = 0.0;
  for (int i = 0; i < static_cast<int>(std::round(seconds / kDt)); ++i) {
    out = update_intent_evidence(s, lambda, force, kDt, p);
  }
  return out;
}
}  // namespace

TEST(ContactIntent, ForceAloneNeverCreatesIntent)
{
  IntentEvidenceParams p;
  IntentEvidenceState s;
  EXPECT_DOUBLE_EQ(run(s, 0.0, 30.0, 5.0, p), 0.0);
  EXPECT_DOUBLE_EQ(intent_compliance_gate(0.0, p), 0.0);
}

TEST(ContactIntent, FirmPushHoldsIntentAfterVisionDrops)
{
  IntentEvidenceParams p;
  IntentEvidenceState s;
  run(s, 1.0, 0.0, 0.5, p);                 // vision says intent before contact
  const double held = run(s, 0.0, 10.0, 5.0, p);   // vision drops, hand pushes
  EXPECT_NEAR(held, 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(intent_compliance_gate(held, p), 1.0);
}

TEST(ContactIntent, ReleaseFadesSmoothlyWithTau)
{
  IntentEvidenceParams p;
  IntentEvidenceState s;
  run(s, 1.0, 10.0, 1.0, p);
  const double after = run(s, 0.0, 0.0, p.release_tau, p);   // ~one tau without force
  const double steps = std::round(p.release_tau / kDt);
  EXPECT_NEAR(after, std::exp(-steps * kDt / p.release_tau), 1e-12);
  // no step anywhere: per-cycle change bounded by dt / tau
  double prev = after;
  for (int i = 0; i < 200; ++i) {
    const double v = update_intent_evidence(s, 0.0, 0.0, kDt, p);
    EXPECT_LE(prev - v, kDt / p.release_tau + 1e-12);
    prev = v;
  }
}

TEST(ContactIntent, LightTouchDecaysSlowerThanNoTouch)
{
  IntentEvidenceParams p;
  IntentEvidenceState a;
  IntentEvidenceState b;
  run(a, 1.0, 0.0, 0.1, p);
  run(b, 1.0, 0.0, 0.1, p);
  const double light = run(a, 0.0, 2.0, 0.5, p);   // hold(2 N) = 0.5
  const double none = run(b, 0.0, 0.0, 0.5, p);
  EXPECT_GT(light, none);
  EXPECT_LT(light, 1.0);
}

TEST(ContactIntent, LambdaAboveMemoryPassesThrough)
{
  IntentEvidenceParams p;
  IntentEvidenceState s;
  EXPECT_DOUBLE_EQ(update_intent_evidence(s, 0.7, 0.0, kDt, p), 0.7);
  EXPECT_DOUBLE_EQ(update_intent_evidence(s, std::nan(""), 0.0, kDt, p) <= 0.7, true);
  EXPECT_THROW(update_intent_evidence(s, 0.5, 0.0, -1.0, p), std::invalid_argument);
}

TEST(ContactIntent, ComplianceGateIsSmoothAndMonotone)
{
  IntentEvidenceParams p;
  double prev = 0.0;
  for (double l = 0.0; l <= 1.0; l += 0.01) {
    const double c = intent_compliance_gate(l, p);
    EXPECT_GE(c, prev - 1e-15);
    prev = c;
  }
  EXPECT_DOUBLE_EQ(intent_compliance_gate(0.5, p), 0.0);   // pre-intent cap of the bridge
  EXPECT_DOUBLE_EQ(intent_compliance_gate(0.8, p), 1.0);
  IntentEvidenceParams bad;
  bad.compliance_high = 0.4;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
}

TEST(ContactIntent, GeometricTargetScaleGivesIntermediateWeights)
{
  const double floor = 0.00025;
  EXPECT_DOUBLE_EQ(contact_target_scale(0.0, floor, true), 1.0);
  EXPECT_NEAR(contact_target_scale(1.0, floor, true), floor, 1e-15);
  EXPECT_NEAR(contact_target_scale(0.5, floor, true), std::sqrt(floor), 1e-12);
  // linear keeps the old formula
  EXPECT_NEAR(contact_target_scale(0.5, floor, false), floor + 0.5 * (1.0 - floor), 1e-15);
  // floor 0: geometric falls back to linear
  EXPECT_NEAR(contact_target_scale(0.5, 0.0, true), 0.5, 1e-15);
}

}  // namespace ur10_intent_admittance_rmp_v2
