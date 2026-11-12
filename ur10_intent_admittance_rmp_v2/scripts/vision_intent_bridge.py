#!/usr/bin/env python3
"""Vision (Stage 5 approach judgment) -> control intent lambda.

Goal 2, step 0 (2026-11-07).  The vision stack publishes, per visible hand,
hand_detector_msgs/ApproachJudgment at ~30 Hz.  The control side calls its
intent value **lambda** (0 = avoid, 1 = comply) so it is not confused with the
vision's own p (hand-approach confidence).

Mapping (all smooth in the vision scores, see VisionIntentBridge.update):
  per hand   lambda_raw = smoothstep((d_intention - on_low) / (on_high - on_low))
             (d_intention = min(p, q) is continuous; 50 is the vision's own
              intention boundary, ties go to "unintentional" -> lambda 0)
  veto       any live hand with d_unintention >= veto_threshold forces 0 and
             cancels the hold (the vision's "uncertain -> unintentional"
             safety direction is kept)
  hands      no veto -> the largest lambda among live hands
  stale      a hand whose last message is older than expire_sec is "no
             information" and is ignored (no message != no approach)
  hold       the vision's intention ends when the hand stops on the cover
             (d_intention drops to 0 at contact), and a new short "run" can
             start with a low p (run15: d_I 96 -> 16 while the gaze q stayed
             > 90).  So the best recent value is held for hold_sec and then
             decayed with time constant decay_tau_sec, but the held value is
             capped by the gaze alone, g_q = smoothstep((q - on_low) / span)
             (min over live hands): looking away lowers lambda at once (the
             vision's latch case), a weak new run does not.
             Holding through the contact itself is the control side's job
             (later step: lambda_eff = max(lambda, g(F))).

Mode "graded" (goal 2, 2026-11-07; mode "judgment" above stays the default):
  per hand   lambda_hand = G(q) * max(P(p), pre_gain * N(dist))
               G(q)    = smoothstep((q - on_low) / (on_high - on_low))   gaze
               P(p)    = smoothstep((p - p_low) / (p_high - p_low))       hand progress
               N(dist) = smoothstep((pre_far - dist) / (pre_far - pre_near))
                         hand-to-forearm distance from the vision (Stage 2)
             So intent starts to rise while the reach is still building up
             (p 20 -> 50 instead of waiting for min(p, q) >= 50), and a hand
             that is merely near the cover while the person looks at the box
             gives at most pre_gain (0.5): enough to soften avoidance, never
             enough on its own to open compliance on the control side.
             q <= 50 (looking away / no face / invalid head pose) gives 0, as
             in the vision's own safety direction.  Veto, hand selection,
             expiry, hold and gaze cap are the same as in "judgment" mode.

Output: std_msgs/Float64 on the controller's existing intent topic
(/worker_intent/compliance_probability) at publish_rate_hz, plus
/worker_intent/lambda_debug = [lambda, lambda_raw_max, veto, live_hands,
held, d_I slot0, d_U slot0, d_I slot1, d_U slot1].

Read-only with respect to the robot: it never commands motion.
"""

import math

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64, Float64MultiArray
    from hand_detector_msgs.msg import ApproachJudgment
except ImportError:  # offline use of VisionIntentBridge only
    rclpy = None
    Node = object


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


class VisionIntentBridge:
    """Pure logic (no ROS) so it can be replayed offline from bags."""

    def __init__(self, on_low=50.0, on_high=80.0, veto_threshold=50.0,
                 expire_sec=0.3, hold_sec=1.0, decay_tau_sec=0.3, mode='judgment',
                 p_low=20.0, p_high=50.0, pre_gain=0.5, pre_near=0.10, pre_far=0.40):
        if not (0.0 <= on_low < on_high <= 100.0):
            raise ValueError('need 0 <= on_low < on_high <= 100')
        if mode not in ('judgment', 'graded'):
            raise ValueError('mode must be judgment or graded')
        if not (0.0 <= p_low < p_high <= 100.0 and 0.0 <= pre_gain <= 1.0 and 0.0 <= pre_near < pre_far):
            raise ValueError('invalid graded-mode parameters')
        self.mode = mode
        self.p_low, self.p_high = p_low, p_high
        self.pre_gain, self.pre_near, self.pre_far = pre_gain, pre_near, pre_far
        if expire_sec <= 0.0 or hold_sec < 0.0 or decay_tau_sec <= 0.0:
            raise ValueError('expire_sec, decay_tau_sec > 0 and hold_sec >= 0')
        self.on_low, self.on_high = on_low, on_high
        self.veto_threshold = veto_threshold
        self.expire_sec, self.hold_sec, self.decay_tau = expire_sec, hold_sec, decay_tau_sec
        self.hands = {}           # slot -> (receive time, d_I, d_U, q, p, dist)
        self.peak = 0.0           # held lambda
        self.peak_t = -math.inf   # time the held value was last refreshed
        self.last = {}

    def on_judgment(self, now, slot, d_intention, d_unintention, q=100.0, p=0.0, dist=math.inf):
        if not (math.isfinite(d_intention) and math.isfinite(d_unintention) and math.isfinite(q)
                and math.isfinite(p)):
            return
        dist = float(dist) if math.isfinite(dist) else math.inf
        self.hands[int(slot)] = (now, float(d_intention), float(d_unintention), float(q), float(p), dist)

    def _hand_lambda(self, v):
        if self.mode == 'judgment':
            return self._ramp(v[1])
        gaze = self._ramp(v[3])
        progress = smoothstep((v[4] - self.p_low) / (self.p_high - self.p_low))
        near = 0.0 if not math.isfinite(v[5]) else \
            smoothstep((self.pre_far - v[5]) / (self.pre_far - self.pre_near))
        return gaze * max(progress, self.pre_gain * near)

    def _ramp(self, score):
        return smoothstep((score - self.on_low) / (self.on_high - self.on_low))

    def update(self, now):
        live = {s: v for s, v in self.hands.items() if now - v[0] <= self.expire_sec}
        veto = any(v[2] >= self.veto_threshold for v in live.values())
        raw = max((self._hand_lambda(v) for v in live.values()), default=0.0)
        gaze_cap = min((self._ramp(v[3]) for v in live.values()), default=1.0)
        if veto:
            self.peak, self.peak_t = 0.0, now
            lam = 0.0
            held = False
        else:
            elapsed = now - self.peak_t
            decayed = self.peak if elapsed <= self.hold_sec else \
                self.peak * math.exp(-(elapsed - self.hold_sec) / self.decay_tau)
            # the gaze cap also lowers what is held, so a later recovery of the
            # gaze cannot bring back an intent the person looked away from
            decayed = min(decayed, gaze_cap)
            if raw >= decayed:
                self.peak, self.peak_t = raw, now
                lam, held = raw, False
            else:
                if decayed < self.peak:
                    # store the lowered value; keep the hold clock where it was
                    # (or at the end of the hold once decaying) so the decay
                    # continues smoothly from here
                    self.peak, self.peak_t = decayed, now - min(elapsed, self.hold_sec)
                lam, held = decayed, True
        self.last = {'lambda': lam, 'raw': raw, 'veto': veto, 'live': len(live), 'held': held,
                     'gaze_cap': gaze_cap, 'slots': {s: (v[1], v[2]) for s, v in live.items()}}
        return lam


class VisionIntentBridgeNode(Node):
    def __init__(self):
        super().__init__('vision_intent_bridge')
        p = self.declare_parameter
        self.bridge = VisionIntentBridge(
            on_low=p('on_low', 50.0).value, on_high=p('on_high', 80.0).value,
            veto_threshold=p('veto_threshold', 50.0).value,
            expire_sec=p('expire_sec', 0.3).value, hold_sec=p('hold_sec', 1.0).value,
            decay_tau_sec=p('decay_tau_sec', 0.3).value, mode=p('mode', 'judgment').value,
            p_low=p('p_low', 20.0).value, p_high=p('p_high', 50.0).value,
            pre_gain=p('pre_gain', 0.5).value, pre_near=p('pre_near_m', 0.10).value,
            pre_far=p('pre_far_m', 0.40).value)
        out_topic = p('output_topic', '/worker_intent/compliance_probability').value
        rate = p('publish_rate_hz', 50.0).value
        self.create_subscription(ApproachJudgment, p('input_topic', '/hand_detector/approach_judgment').value,
                                 self._on_judgment, 10)
        self.pub = self.create_publisher(Float64, out_topic, 10)
        self.dbg = self.create_publisher(Float64MultiArray, '/worker_intent/lambda_debug', 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().warn(
            f'vision intent bridge: lambda -> {out_topic} at {rate:.0f} Hz '
            f'(on {self.bridge.on_low:.0f}..{self.bridge.on_high:.0f}, veto d_U >= '
            f'{self.bridge.veto_threshold:.0f}, expire {self.bridge.expire_sec:.2f} s, '
            f'hold {self.bridge.hold_sec:.2f} s, decay {self.bridge.decay_tau:.2f} s, mode {self.bridge.mode}'
            + (f': p {self.bridge.p_low:.0f}..{self.bridge.p_high:.0f}, pre {self.bridge.pre_gain:.2f} x '
               f'near {self.bridge.pre_near:.2f}..{self.bridge.pre_far:.2f} m' if self.bridge.mode == 'graded' else '')
            + ')')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_judgment(self, m):
        # receive time, not the camera stamp: the two machines' clocks are not
        # guaranteed to agree, and expiry must reflect what the controller sees.
        self.bridge.on_judgment(self._now(), m.hand_slot, m.d_intention, m.d_unintention, m.q,
                                m.p_approach, m.hand_robot_distance)

    def _tick(self):
        lam = self.bridge.update(self._now())
        self.pub.publish(Float64(data=float(lam)))
        last = self.bridge.last
        slots = last['slots']
        values = [float(lam), float(last['raw']), float(last['veto']), float(last['live']),
                  float(last['held'])]
        for s in (0, 1):
            values += [float(v) for v in slots.get(s, (math.nan, math.nan))]
        d = Float64MultiArray()
        d.data = values
        self.dbg.publish(d)


def main():
    rclpy.init()
    node = VisionIntentBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
