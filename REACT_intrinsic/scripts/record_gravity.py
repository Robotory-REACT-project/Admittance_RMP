#!/usr/bin/env python3
"""중력보상 계수용 기록기. 센서를 누르지 않은 채 팔을 기울이며 관절값과 F/T 를 남긴다.

센서가 달린 3번 팔(forearm)의 기울기는 어깨 들기(shoulder_lift)와
팔꿈치(elbow) 두 관절의 합 phi = theta2 + theta3 하나로 정해진다.
어깨 회전(수직축)과 손목 세 관절은 센서가 느끼는 무게 방향을 바꾸지 않는다.
그래서 센서 6축은 각각 a*cos(phi) + b*sin(phi) + c 로 적합할 수 있고,
질량·무게중심·장착 방향을 따로 잴 필요가 없다.

2026-09-22 부터 FTGetMain 이 이 방식으로 중력보상을 직접 수행한다.
보상이 걸린 채로 기록해도 된다. 모델이 cos, sin 에 대해 선형이므로,
보상 후 남은 값에 같은 식을 맞추면 그 결과가 곧 계수 보정량이다
(새 계수 = 코드의 계수 + 잔차에서 적합한 계수).
보상이 잘 되고 있으면 남은 값은 자세와 상관없이 0 근처에 머문다.

쓰는 법:
    ros2 launch Y2FT_AQ ft_get_main.launch.py
    python3 record_gravity.py

    이름 치고 Enter  -> 팔을 천천히 기울인다 -> Enter 로 종료
    q + Enter        -> csv 저장하고 끝

    이름 예)  sweep1, sweep2   (같은 동작을 두 번 해서 재현되는지 본다)
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

SAMPLE_HZ = 50.0       # 기록 주기. 천천히 기울이는 동작이라 이 정도면 충분하다
STALE_SEC = 0.2        # 이보다 오래된 값이 섞이면 그 줄은 버린다
MIN_SPAN_DEG = 90.0    # phi 를 이만큼은 쓸어야 cos 항과 상수가 구분된다

# UR 드라이버마다 앞에 접두어가 붙기도 해서 이름의 일부로 찾는다.
JOINT_KEYS = ('shoulder_pan', 'shoulder_lift', 'elbow',
              'wrist_1', 'wrist_2', 'wrist_3')


class GravityRecorder(Node):
    def __init__(self, joint_topic, ft_topics):
        super().__init__('record_gravity')
        self.rows = []
        self.active = False
        self.label = ''
        self.t0 = 0.0
        self.joints = None
        self.joint_time = 0.0
        self.ft = [None] * len(ft_topics)
        self.ft_time = [0.0] * len(ft_topics)
        self.joint_index = None
        self.create_subscription(JointState, joint_topic, self._joint, 50)
        for i, topic in enumerate(ft_topics):
            self.create_subscription(
                WrenchStamped, topic, lambda m, i=i: self._ft(i, m), 50)
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

    def _ft(self, i, msg):
        w = msg.wrench
        self.ft[i] = [w.force.x, w.force.y, w.force.z,
                      w.torque.x, w.torque.y, w.torque.z]
        self.ft_time[i] = time.time()

    def fresh(self):
        now = time.time()
        return (self.joints is not None
                and now - self.joint_time < STALE_SEC
                and all(v is not None for v in self.ft)
                and all(now - t < STALE_SEC for t in self.ft_time))

    def phi_deg(self):
        return math.degrees(self.joints[1] + self.joints[2])

    def _sample(self):
        if not self.active or not self.fresh():
            return
        row = [self.label, time.time() - self.t0, self.phi_deg(), *self.joints]
        for values in self.ft:
            row.extend(values)
        self.rows.append(row)

    def start(self, label):
        self.label = label
        self.t0 = time.time()
        self.active = True

    def stop(self):
        self.active = False


def norm3(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def live_print(node, stop_event):
    """기록 중 1초마다 현재 phi 와 힘 크기를 보여준다."""
    while not stop_event.wait(1.0):
        if not node.fresh():
            print('     (관절값 또는 F/T 가 안 들어옵니다)')
            continue
        forces = '  '.join(
            f'|F{i + 1}| {norm3(v):5.2f} N' for i, v in enumerate(node.ft))
        print(f'     phi {node.phi_deg():+7.1f} deg   {forces}   '
              f'{len(node.rows)} 줄')


def summarize(rows, sensor_count):
    if len(rows) < 10:
        return f'   {len(rows)} 줄뿐이라 평가 생략'
    phis = [r[2] for r in rows]
    span = max(phis) - min(phis)
    lines = [f'   {len(rows)} 줄,  phi {min(phis):+.1f} ~ {max(phis):+.1f} deg '
             f'(쓴 범위 {span:.1f} deg)']
    if span < MIN_SPAN_DEG:
        lines.append(f'   !! {MIN_SPAN_DEG:.0f} deg 보다 좁습니다. 적합이 불안정할 수 있어요.')
    # 무게가 회전하면 힘의 크기는 그대로이고 축별 성분만 바뀐다.
    # 그래서 크기 대신 축마다 얼마나 움직였는지를 본다.
    names = ('Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz')
    for s in range(sensor_count):
        base = 9 + 6 * s
        spans = []
        for k, name in enumerate(names):
            values = [r[base + k] for r in rows]
            unit = 'N' if k < 3 else 'Nm'
            digits = 2 if k < 3 else 3
            spans.append(f'{name} {max(values) - min(values):.{digits}f}{unit}')
        lines.append(f'   센서{s + 1} 변화폭  ' + '  '.join(spans))
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--joint-topic', default='/UR10_right/joint_states')
    parser.add_argument('--ft-topics', nargs='+',
                        default=['/ur10skku/ftdata_tcp_1', '/ur10skku/ftdata_tcp_2'])
    parser.add_argument('--out', default=None)
    args = parser.parse_args()

    out_path = args.out or os.path.expanduser(
        '~/gravity_{}.csv'.format(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))

    rclpy.init()
    node = GravityRecorder(args.joint_topic, args.ft_topics)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    print(f'관절 토픽: {args.joint_topic}')
    print(f'F/T 토픽 : {", ".join(args.ft_topics)}')
    time.sleep(1.5)

    if not node.fresh():
        print('!! 관절값 또는 F/T 값이 아직 안 들어옵니다. 토픽을 확인하세요.\n')
    else:
        print(f'지금 phi = {node.phi_deg():+.1f} deg\n')

    print('센서를 누르지 말고, 팔을 천천히 기울이세요.')
    print('  이름 + Enter 로 시작, 다 움직이면 Enter 로 종료.  q + Enter 로 전체 종료.\n')
    runs = 0
    try:
        while True:
            label = input('이름 (q=종료) > ').strip()
            if label.lower() == 'q':
                break
            if not label or ',' in label:
                print('   이름을 쳐주세요 (쉼표 없이). 예: sweep1\n')
                continue
            before = len(node.rows)
            stop_event = threading.Event()
            printer = threading.Thread(
                target=live_print, args=(node, stop_event), daemon=True)
            node.start(label)
            printer.start()
            input('   기록 중... 다 움직이면 Enter > ')
            node.stop()
            stop_event.set()
            printer.join()
            print(summarize(node.rows[before:], len(args.ft_topics)) + '\n')
            runs += 1
    except (KeyboardInterrupt, EOFError):
        node.stop()

    header = ['label', 't_sec', 'phi_deg',
              'q_pan', 'q_lift', 'q_elbow', 'q_w1', 'q_w2', 'q_w3']
    for s in range(len(args.ft_topics)):
        header += [f's{s + 1}_{k}' for k in ('Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz')]
    with open(out_path, 'w', encoding='utf-8') as h:
        h.write(','.join(header) + '\n')
        for r in node.rows:
            h.write(r[0] + ',' + ','.join(f'{v:.6f}' for v in r[1:]) + '\n')
    print(f'\n저장: {out_path}   (총 {len(node.rows)} 줄, {runs} 회)')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
