#!/usr/bin/env python3
"""중력보상 장시간 시험 기록기. 누르지 않은 채 로봇을 오래 움직이며 관절값·F/T·접촉 판정을 남긴다.

record_gravity.py 와 다른 점
    - Enter 로 구간을 나누지 않는다. 켜면 끝날 때까지 쭉 기록한다 (Ctrl+C 또는 --minutes)
    - 1초마다 파일에 써서 중간에 꺼져도 그때까지는 남는다
    - main_ifs 의 contact_state, force_magnitude 도 같이 남긴다 (안 눌렀는데 접촉으로 잡혔는지)
    - 관절값·F/T 가 끊긴 시간도 센다

쓰는 법:
    ros2 launch Y2FT_AQ ft_get_main.launch.py        # 로봇 정지 상태로, 20초 기다린 뒤
    ros2 launch REACT_intrinsic react_intrinsic_JSshape.launch.py
    python3 record_gravity_long.py --minutes 35

    분석: python3 ~/ros2_ws/docs/plot_gravity_long.py <csv> --phi0 <로그의 gravity reference 각도>
"""
import argparse
import datetime
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

SAMPLE_HZ = 50.0       # 기록 주기
STALE_SEC = 0.2        # 이보다 오래된 관절값·F/T 가 섞이면 그 줄은 쓰지 않고 '끊김' 으로 센다
STATUS_SEC = 10.0      # 화면 표시 주기

JOINT_KEYS = ('shoulder_pan', 'shoulder_lift', 'elbow',
              'wrist_1', 'wrist_2', 'wrist_3')
SENSORS = (1, 2)
NAN = float('nan')


class LongRecorder(Node):
    def __init__(self, joint_topic):
        super().__init__('record_gravity_long')
        self.lock = threading.Lock()
        self.joints = None
        self.joint_time = 0.0
        self.joint_index = None
        self.ft = {s: None for s in SENSORS}
        self.ft_time = {s: 0.0 for s in SENSORS}
        self.contact = {s: NAN for s in SENSORS}
        self.mag = {s: NAN for s in SENSORS}
        self.contact_seen = {s: False for s in SENSORS}
        self.rows = []
        self.stale = 0
        self.t0 = None
        self.create_subscription(JointState, joint_topic, self._joint, 50)
        for s in SENSORS:
            self.create_subscription(
                WrenchStamped, f'/ur10skku/ftdata_tcp_{s}',
                lambda m, s=s: self._ft(s, m), 50)
            ns = f'/sensor{s}/contactsensing'
            self.create_subscription(Bool, ns + '/contact_state',
                                     lambda m, s=s: self._contact(s, m), 50)
            self.create_subscription(Float64, ns + '/force_magnitude',
                                     lambda m, s=s: self._mag(s, m), 50)
        self.create_timer(1.0 / SAMPLE_HZ, self._sample)

    def _joint(self, msg):
        if self.joint_index is None:
            index = []
            for key in JOINT_KEYS:
                found = [k for k, name in enumerate(msg.name) if key in name]
                if not found:
                    self.get_logger().error(
                        f"관절 '{key}' 를 못 찾았습니다. 받은 이름: {list(msg.name)}")
                    return
                index.append(found[0])
            self.joint_index = index
        self.joints = [msg.position[k] for k in self.joint_index]
        self.joint_time = time.time()

    def _ft(self, s, msg):
        w = msg.wrench
        self.ft[s] = [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]
        self.ft_time[s] = time.time()

    def _contact(self, s, msg):
        self.contact[s] = 1.0 if msg.data else 0.0
        self.contact_seen[s] = True

    def _mag(self, s, msg):
        self.mag[s] = msg.data

    def fresh(self):
        now = time.time()
        return (self.joints is not None and now - self.joint_time < STALE_SEC
                and all(self.ft[s] is not None and now - self.ft_time[s] < STALE_SEC
                        for s in SENSORS))

    def phi_deg(self):
        return math.degrees(self.joints[1] + self.joints[2])

    def _sample(self):
        if self.t0 is None:
            return
        if not self.fresh():
            with self.lock:
                self.stale += 1
            return
        now = time.time()
        row = [now - self.t0, now, self.phi_deg(), *self.joints]
        for s in SENSORS:
            row += self.ft[s]
        row += [self.contact[s] for s in SENSORS] + [self.mag[s] for s in SENSORS]
        with self.lock:
            self.rows.append(row)

    def take_rows(self):
        with self.lock:
            rows, self.rows = self.rows, []
        return rows


def header():
    h = ['t_sec', 'wall_time', 'phi_deg', 'q_pan', 'q_lift', 'q_elbow', 'q_w1', 'q_w2', 'q_w3']
    for s in SENSORS:
        h += [f's{s}_{k}' for k in ('Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz')]
    h += [f's{s}_contact' for s in SENSORS] + [f's{s}_mag' for s in SENSORS]
    return h


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--joint-topic', default='/UR10_right/joint_states')
    parser.add_argument('--minutes', type=float, default=35.0, help='이 시간이 지나면 저절로 끝')
    parser.add_argument('--out', default=None)
    args = parser.parse_args()
    out_path = args.out or os.path.expanduser('~/gravity_long_{}.csv'.format(
        datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))

    rclpy.init()
    node = LongRecorder(args.joint_topic)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    time.sleep(2.0)
    if not node.fresh():
        print('!! 관절값 또는 F/T 가 안 들어옵니다. FTGetMain 과 UR 드라이버를 확인하세요.')
    else:
        print(f'지금 phi = {node.phi_deg():+.1f} deg')
    for s in SENSORS:
        if not node.contact_seen[s]:
            print(f'!! 센서{s} contact_state 가 안 들어옵니다 (main_ifs 확인). '
                  '기록은 하지만 헛접촉은 셀 수 없습니다.')
    print(f'\n저장: {out_path}')
    print(f'{args.minutes:g} 분 기록합니다. 센서를 누르지 마세요. 먼저 끝내려면 Ctrl+C\n')

    n_rows = 0
    n_contact_rows = {s: 0 for s in SENSORS}
    with open(out_path, 'w', encoding='utf-8') as h:
        h.write(','.join(header()) + '\n')
        node.t0 = time.time()
        last_status = 0.0
        try:
            while time.time() - node.t0 < args.minutes * 60.0:
                time.sleep(1.0)
                rows = node.take_rows()
                for r in rows:
                    h.write(','.join(f'{v:.6f}' for v in r) + '\n')
                h.flush()
                n_rows += len(rows)
                for r in rows:
                    for i, s in enumerate(SENSORS):
                        if r[-4 + i] == 1.0:
                            n_contact_rows[s] += 1
                el = time.time() - node.t0
                if el - last_status >= STATUS_SEC and node.fresh():
                    last_status = el
                    f = '  '.join(
                        f'|F{s}| {math.sqrt(sum(v * v for v in node.ft[s][:3])):5.2f} N'
                        for s in SENSORS)
                    c = '  '.join(f'접촉{s} {n_contact_rows[s] / SAMPLE_HZ:5.1f} s' for s in SENSORS)
                    print(f'  {el / 60:5.1f} 분  phi {node.phi_deg():+7.1f}°  {f}  | {c}'
                          f'  | 끊김 {node.stale / SAMPLE_HZ:.1f} s')
        except KeyboardInterrupt:
            pass
        rows = node.take_rows()
        for r in rows:
            h.write(','.join(f'{v:.6f}' for v in r) + '\n')
        n_rows += len(rows)

    print(f'\n저장: {out_path}  ({n_rows} 줄, {n_rows / SAMPLE_HZ / 60:.1f} 분, '
          f'끊김 {node.stale / SAMPLE_HZ:.1f} s)')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
