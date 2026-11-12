#!/usr/bin/env python3
"""단축어 모드 실시간 노드. 트리거 → 5초 글씨 → O/X 판정 → 토픽 발행.

    python3 shortcut_node.py                  # 로봇 + main_ifs 두 개가 돌고 있을 때
    python3 shortcut_node.py --assume-still   # 로봇 없이 책상 시험. 관절값을 안 보고 늘 멈춤으로 본다
                                              # (로봇에서는 쓰지 말 것)

구독: /sensor{1,2}/contactsensing/cp, contact_state, force_magnitude
      → record_gesture.py 와 같게 100 Hz 타이머로 최신값을 묶는다 (학습 데이터와 같은 조건)
      /UR10_right/joint_states  → 정지 판단
발행: /shortcut/mode     std_msgs/Bool    모드 켜짐 여부 (바뀔 때 + 1초마다)
      /shortcut/command  std_msgs/String  "O" 또는 "X" (판정될 때 한 번. 기타면 안 나감)

판단 로직은 ~/ros2_ws/docs/ 의 shortcut_core.py (트리거·모드), gesture_classifier.py (O/X 판정) 를
그대로 쓴다. 학습 코드와 같은 파일을 쓰려고 docs 를 import 경로에 넣는다 (--docs-dir).
소스에서 직접 실행한다 (설치 안 함).
"""
import argparse
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

SAMPLE_HZ = 100.0          # main_ifs 발행 주기, record_gesture.py 와 같게
STILL_MAX_RAD_S = 0.01     # 관절 속도가 모두 이보다 작으면 멈춤
STILL_HOLD_SEC = 0.5       # 이만큼 계속 멈춰 있어야 정지로 본다
JOINT_TIMEOUT_SEC = 0.5    # 관절값이 이보다 오래 안 오면 움직이는 것으로 본다 (안전한 쪽)
MODE_REPEAT_SEC = 1.0      # /shortcut/mode 를 바뀌지 않아도 이 주기로 다시 보낸다
DEFAULT_DOCS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'docs')


class SensorState:
    def __init__(self):
        self.cp = (0.0, 0.0, 0.0)
        self.contact = False
        self.mag = float('nan')
        self.cp_count = 0


class ShortcutNode(Node):
    def __init__(self, core, mode, assume_still, joint_topic):
        super().__init__('shortcut_mode')
        self.core = core
        self.mode = mode
        self.assume_still = assume_still
        self.state = {s: SensorState() for s in core.SENSORS}
        now = time.monotonic()
        self.last_joint = -1e9
        self.last_moving = now
        self.prev_joint = None          # (시각, 위치) — 속도가 비어 있을 때 위치 차로 계산
        self.was_still = None
        self.last_mode_pub = 0.0

        for s in core.SENSORS:
            ns = f'/sensor{s}/contactsensing'
            st = self.state[s]
            self.create_subscription(PointStamped, ns + '/cp',
                                     lambda m, st=st: self._cp(st, m), 100)
            self.create_subscription(Bool, ns + '/contact_state',
                                     lambda m, st=st: setattr(st, 'contact', m.data), 100)
            self.create_subscription(Float64, ns + '/force_magnitude',
                                     lambda m, st=st: setattr(st, 'mag', m.data), 100)
        if not assume_still:
            self.create_subscription(JointState, joint_topic, self._joint, qos_profile_sensor_data)
        self.mode_pub = self.create_publisher(Bool, '/shortcut/mode', 10)
        self.cmd_pub = self.create_publisher(String, '/shortcut/command', 10)
        self.create_timer(1.0 / SAMPLE_HZ, self._tick)
        self.create_timer(3.0, self._check_inputs)
        self.get_logger().info(
            f'단축어 모드 대기 중. 트리거: {type(mode.trigger).__name__}, '
            f'글씨 {core.WINDOW_SEC:g} s, 정지 판단: '
            + ('끔 (--assume-still, 책상 시험용)' if assume_still else joint_topic))

    def _cp(self, st, m):
        st.cp = (m.point.x, m.point.y, m.point.z)
        st.cp_count += 1

    def _joint(self, m):
        now = time.monotonic()
        self.last_joint = now
        if m.velocity:
            speed = max(abs(v) for v in m.velocity)
        elif self.prev_joint is not None and now > self.prev_joint[0]:
            dt = now - self.prev_joint[0]
            speed = max(abs(a - b) / dt for a, b in zip(m.position, self.prev_joint[1]))
        else:
            speed = float('inf')
        self.prev_joint = (now, list(m.position))
        if speed > STILL_MAX_RAD_S:
            self.last_moving = now

    def _still(self, now):
        if self.assume_still:
            return True
        return (now - self.last_joint <= JOINT_TIMEOUT_SEC
                and now - self.last_moving >= STILL_HOLD_SEC)

    def _publish_mode(self, now):
        self.mode_pub.publish(Bool(data=self.mode.active))
        self.last_mode_pub = now

    def _tick(self):
        now = time.monotonic()
        still = self._still(now)
        if still != self.was_still:
            self.get_logger().info('로봇 정지 — 트리거 받음' if still else '로봇 움직임 — 트리거 안 받음')
            self.was_still = still
        samples = {s: (st.cp, st.contact, st.mag) for s, st in self.state.items()}
        for ev in self.mode.step(now, samples, still):
            if ev['type'] == 'mode':
                self.get_logger().info(('모드 켜짐: ' if ev['on'] else '모드 꺼짐: ') + ev['reason'])
                self._publish_mode(now)
            elif ev['type'] == 'result':
                probs = ('O {:.2f} X {:.2f}'.format(*ev['probs'])
                         if ev['probs'] is not None else '')
                self.get_logger().info(f'판정 {ev["label"]} (센서{ev["sensor"]}, '
                                       f'{ev["size"]:.0f} mm, {probs}) — {ev["why"]}')
            elif ev['type'] == 'command':
                self.cmd_pub.publish(String(data=ev['label']))
                self.get_logger().info(f'/shortcut/command 발행: {ev["label"]}')
        if now - self.last_mode_pub >= MODE_REPEAT_SEC:
            self._publish_mode(now)

    def _check_inputs(self):
        for s, st in self.state.items():
            if st.cp_count == 0:
                self.get_logger().warn(f'센서{s} cp 가 안 들어옴 (main_ifs 확인)')
        if not self.assume_still and self.last_joint < 0:
            self.get_logger().warn('관절값이 안 들어옴 — 정지로 보지 않으므로 트리거를 안 받음')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--docs-dir', default=DEFAULT_DOCS,
                        help='shortcut_core.py, gesture_classifier.py 가 있는 폴더')
    parser.add_argument('--model', default=None, help='기본: gesture_classifier.DEFAULT_MODEL')
    parser.add_argument('--joint-topic', default='/UR10_right/joint_states')
    parser.add_argument('--assume-still', action='store_true',
                        help='관절값을 안 보고 늘 정지로 본다 (책상 시험용)')
    args, ros_args = parser.parse_known_args()

    sys.path.insert(0, os.path.realpath(args.docs_dir))
    import gesture_classifier as gc
    import shortcut_core as core

    clf = gc.GestureClassifier(args.model or gc.DEFAULT_MODEL)   # 켤 때 미리 불러 둔다
    mode = core.ShortcutMode(clf, core.DoubleSwipeTrigger())

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = ShortcutNode(core, mode, args.assume_still, args.joint_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
