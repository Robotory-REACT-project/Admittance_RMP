#!/usr/bin/env python3
"""lambda ramp for the measured metric-authority curve (stage 9, no vision).

Publishes lambda on /worker_intent/compliance_probability at 50 Hz, never pausing
(the controller treats a 0.5 s gap as lambda = 0).

  Enter      countdown, then lambda ramps 0 -> 1 linearly over --ramp-sec and holds 1
  Enter      (while holding 1) lambda back to 0 -- release the hand first
  q + Enter  quit (publishing stops; controller falls back to lambda = 0)

Why only upward: while the cover is pushed (>= 3 N) the controller's intent
memory holds lambda_eff at its peak, so lambda_eff follows lambda only while it
rises.  Push the cover firmly (about 10 N, at least 6 N) for the whole ramp and
keep following the arm once it starts to yield (lambda_eff above about 0.5).

Each ramp start / end is also published on /experiment/scene (std_msgs/String,
"label|wall_time") so it is stored in the bag next to the robot data.

Usage:
  python3 lambda_ramp.py                       # 4 s ramp, 3 s countdown
  python3 lambda_ramp.py --ramp-sec 5 --delay 2
"""
import argparse
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


class LambdaRamp(Node):
    def __init__(self, ramp_sec, rate_hz):
        super().__init__('lambda_ramp')
        self.ramp_sec = ramp_sec
        self.pub = self.create_publisher(Float64, '/worker_intent/compliance_probability', 10)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene = self.create_publisher(String, '/experiment/scene', qos)
        self.lock = threading.Lock()
        self.ramp_start = None      # monotonic time the ramp started, None = idle
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

    def value(self):
        with self.lock:
            if self.ramp_start is None:
                return 0.0
            return min(max((time.monotonic() - self.ramp_start) / self.ramp_sec, 0.0), 1.0)

    def tick(self):
        self.pub.publish(Float64(data=float(self.value())))

    def start(self):
        with self.lock:
            self.ramp_start = time.monotonic()
        self.mark('lambda_ramp_start')

    def stop(self):
        with self.lock:
            self.ramp_start = None
        self.mark('lambda_zero')

    def mark(self, label):
        self.scene.publish(String(data=f'{label}|{time.time():.3f}'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ramp-sec', type=float, default=4.0)
    parser.add_argument('--delay', type=float, default=3.0, help='countdown before the ramp (s)')
    parser.add_argument('--rate', type=float, default=50.0)
    args = parser.parse_args()
    if not (args.ramp_sec > 0 and args.delay >= 0 and args.rate > 0):
        parser.error('ramp-sec and rate must be > 0, delay >= 0')

    rclpy.init()
    node = LambdaRamp(args.ramp_sec, args.rate)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin)
    spin.start()
    print(f'lambda = 0 published at {args.rate:.0f} Hz. Push the sensor-3 face upward (>= 6 N), then:')
    print('  Enter = start ramp,  Enter again = lambda 0 (after releasing),  q = quit')
    ramping = False
    try:
        while True:
            line = input('[ramping/holding 1] > ' if ramping else '[lambda 0] > ').strip().lower()
            if line == 'q':
                break
            if not ramping:
                for k in range(int(round(args.delay)), 0, -1):
                    print(f'  {k} ...', flush=True)
                    time.sleep(1.0)
                node.start()
                print(f'  ramp 0 -> 1 over {args.ramp_sec:.1f} s  (keep pushing, follow the arm)')
                ramping = True
            else:
                node.stop()
                print('  lambda = 0')
                ramping = False
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        # stop spinning before tearing anything down, then let the thread finish
        executor.shutdown(timeout_sec=1.0)
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
