#!/usr/bin/env python3
"""Scene marker for poster experiments (read-only w.r.t. the robot).

Press Enter to mark the start of the next scene; type a label and Enter to
mark a custom one; 'q' quits.  Each mark is published once on
/experiment/scene (std_msgs/String, "label|wall_time") and printed, so it is
stored in the rosbag next to the robot data.
"""
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

SCENES = ['(a) 스쳐 지나감 - 딴 데 보며 커버 근처 통과',
          '(b) 가만히 - 손 보이게 정지',
          '(c) 일부러 뻗음 - 박스 보며 천천히, 닿지 않음',
          '(d) 커버를 밂 - 박스 보며 밀고 몇 초 뒤 놓기',
          '(e) 대조 - 딴 데 보며 커버를 밂']


def main():
    rclpy.init()
    node = Node('experiment_scene_marker')
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, '/experiment/scene', qos)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    idx = 0
    print('Enter = 다음 장면 표시, 직접 입력 + Enter = 임의 표시, q = 종료')
    for i, s in enumerate(SCENES):
        print(f'  {i + 1}. {s}')
    while True:
        try:
            line = input(f'[다음: {SCENES[idx % len(SCENES)]}] > ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line == 'q':
            break
        label = line if line else SCENES[idx % len(SCENES)]
        if not line:
            idx += 1
        now = time.time()
        pub.publish(String(data=f'{label}|{now:.3f}'))
        print(f'  {time.strftime("%H:%M:%S")}  표시: {label}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
