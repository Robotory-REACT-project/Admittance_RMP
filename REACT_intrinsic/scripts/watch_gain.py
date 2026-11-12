#!/usr/bin/env python3
"""한 센서의 접촉 로그만 골라서 보기 좋게 찍는다.

노드가 /rosout 으로도 보내는 INFO 줄을 받아, 원하는 센서 것만 걸러
접촉점·힘·배율·면 판정을 한 줄로 정리한다. 면 구분(옆면/윗면)이 제대로
되는지, 어느 면의 보정이 걸렸는지 눈으로 확인하는 용도.

모서리 넘어가기 대책(ifs_algorithm.cpp 의 kEdgeTransitionFix)이 켜져 있으면
라벨이 'blend top/side/edge' 이고, 세 숫자는 배율이 아니라 윗면·옆면·모서리
확률(합 1)이다. 이때 면 칸은 접촉점 위치로 윗면/모서리/옆면 셋 중 하나를 쓴다.

쓰는 법:
    python3 watch_gain.py            # 센서 1
    python3 watch_gain.py --sensor 2
    python3 watch_gain.py --node 다른노드이름
"""
import argparse
import re

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# ifs_algorithm.cpp 의 kSideFaceMinZ 와 같은 값이어야 한다.
SIDE_FACE_MIN_Z = -0.013
# ifs_algorithm.cpp 의 kFilletCenterY. 이보다 크고 z < SIDE_FACE_MIN_Z 면 둥근 모서리.
FILLET_MIN_Y = 0.062

# 로그 예)  ... |F|=12.345 N (raw 12.300 N, gain top Mx/My 1.447 1.068 1.000)
#           ... |F|=12.345 N (raw 12.300 N, gain side Fy/Mx/Mz 1.055 1.549 1.033)
#           ... |F|=12.345 N (raw 12.300 N, gain blend top/side/edge 0.950 0.000 0.050)
PATTERN = re.compile(
    r'Contact: \[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\].*?'
    r'\|F\|=([\d.]+) N \(raw ([\d.]+) N, gain (.+?) ([-\d.]+) ([-\d.]+) ([-\d.]+)\)')


class Watch(Node):
    def __init__(self, node_name):
        super().__init__('watch_gain')
        self.target = node_name
        self.count = 0
        qos = QoSProfile(
            depth=100,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Log, '/rosout', self._cb, qos)

    def _cb(self, message):
        if self.target not in message.name:
            return
        found = PATTERN.search(message.msg)
        if not found:
            return
        x, y, z, mag, raw = (float(v) for v in found.groups()[:5])
        label = found.group(6)
        a, b, c = (float(v) for v in found.groups()[6:])
        face = '옆면' if z >= SIDE_FACE_MIN_Z else '윗면'
        if self.count % 20 == 0:
            print(f"\n{'면':>4} {'x[mm]':>8} {'y[mm]':>8} {'z[mm]':>8} "
                  f"{'|F|[N]':>8}   {'보정':<18} {'배율':>20}   판정")
        self.count += 1
        if label.startswith('blend'):
            # 세 식을 확률로 섞으므로 "면 불일치" 는 따지지 않는다.
            if face == '윗면' and y > FILLET_MIN_Y:
                face = '모서리'
            print(f"{face:>4} {x * 1000:>8.1f} {y * 1000:>8.1f} {z * 1000:>8.1f} "
                  f"{mag:>8.2f}   {label:<18} 윗{a:>5.2f} 옆{b:>5.2f} 모{c:>5.2f}")
            return
        # 배율을 고른 면(직전 접촉점 기준)과 최종 접촉점의 면이 다르면
        # 모서리 근처에서 면이 오락가락한다는 뜻이다.
        picked_top = label.startswith('top')
        picked_side = label.startswith('side')
        odd = (face == '윗면' and picked_side) or (face == '옆면' and picked_top)
        print(f"{face:>4} {x * 1000:>8.1f} {y * 1000:>8.1f} {z * 1000:>8.1f} "
              f"{mag:>8.2f}   {label:<18} {a:>6.3f} {b:>6.3f} {c:>6.3f}   "
              f"{'<-- 면 불일치' if odd else ''}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor', type=int, default=1, choices=(1, 2))
    parser.add_argument('--node', default=None,
                        help='기본값: jsshape_sensor<번호>_contactsensing')
    args = parser.parse_args()

    node_name = args.node or f'jsshape_sensor{args.sensor}_contactsensing'
    rclpy.init()
    node = Watch(node_name)
    print(f"'{node_name}' 의 접촉 로그만 봅니다. 눌러보세요. (Ctrl+C 로 종료)")
    print(f"면 경계: z = {SIDE_FACE_MIN_Z * 1000:.0f} mm  (이보다 작으면 윗면)")
    print("blend 줄의 세 숫자는 윗면·옆면·모서리 확률입니다 (모서리 넘어가기 대책).")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
