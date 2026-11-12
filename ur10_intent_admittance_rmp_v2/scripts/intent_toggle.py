#!/usr/bin/env python3
"""Manual intent switch for v2 stage-4 tests.

Publishes p on /worker_intent/compliance_probability continuously (30 Hz).
  Enter        toggle p between 0 and 1
  0.6 + Enter  set any value in [0, 1]
  q + Enter    quit (publishing stops; the controller falls back to p = 0
               after its 0.5 s intent timeout, i.e. avoidance ON)

Publishing never pauses while the value changes, so switching p does not
create the stale-intent gap that restarting `ros2 topic pub` does.

Usage:
  python3 intent_toggle.py              # starts at p = 0 (avoidance ON)
  python3 intent_toggle.py --start 1    # starts at p = 1
"""
import argparse
import sys
import threading

import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Float64


def describe(p):
    # Mirrors stage 4: avoidance weight w = 1 - p (floor 0), admittance can
    # only engage at p >= 0.6 (internal_admittance_enter_probability).
    text = f'avoidance weight w = {1.0 - p:.2f}'
    if p >= 1.0:
        text = 'avoidance OFF (w = 0)'
    elif p <= 0.0:
        text = 'avoidance ON  (w = 1)'
    if p >= 0.6:
        text += ', admittance can engage on contact'
    return text


class IntentToggle(Node):
    def __init__(self, topic, start, rate_hz):
        super().__init__('intent_toggle')
        self._lock = threading.Lock()
        self._value = start
        self._pub = self.create_publisher(Float64, topic, 10)
        self.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self):
        with self._lock:
            value = self._value
        self._pub.publish(Float64(data=value))

    def get(self):
        with self._lock:
            return self._value

    def set(self, value):
        with self._lock:
            self._value = value


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--start', type=float, default=0.0, help='initial p (default 0)')
    parser.add_argument('--topic', default='/worker_intent/compliance_probability')
    parser.add_argument('--rate', type=float, default=30.0, help='publish rate in Hz')
    args = parser.parse_args()
    if not 0.0 <= args.start <= 1.0:
        parser.error('--start must be within [0, 1]')

    rclpy.init()
    node = IntentToggle(args.topic, args.start, args.rate)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    print(f'publishing on {args.topic} at {args.rate:.0f} Hz')
    print('Enter = toggle 0/1, a number = set value, q = quit')
    print(f'p = {node.get():.2f}  {describe(node.get())}', flush=True)
    try:
        for line in sys.stdin:
            text = line.strip().lower()
            if text in ('q', 'quit', 'exit'):
                break
            if text == '':
                new = 0.0 if node.get() >= 0.5 else 1.0
            else:
                try:
                    new = float(text)
                except ValueError:
                    print('  type Enter, a number in [0, 1], or q', flush=True)
                    continue
                if not 0.0 <= new <= 1.0:
                    print('  value must be within [0, 1]', flush=True)
                    continue
            node.set(new)
            print(f'p = {new:.2f}  {describe(new)}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the spinning thread before tearing the node down.
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
        print('stopped publishing: the controller treats p as 0 after 0.5 s (avoidance ON)')


if __name__ == '__main__':
    main()
