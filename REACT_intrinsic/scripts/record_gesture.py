#!/usr/bin/env python3
"""O/X 글씨 기록기. 커버에 손가락으로 쓴 글자 하나의 접촉점 궤적과 F/T 를 남긴다.

글씨 인식 학습 데이터용이다. record_drag.py 와 달리 약한 힘과 손을 뗀 구간도
버리지 않는다 (X 의 두 획을 나누려면 뗀 구간이 있어야 한다). 센서 1, 2 를 같이 남긴다.

쓰는 법:
    python3 record_gesture.py --face top  --writer kth
    python3 record_gesture.py --face side --writer kth

    o + Enter  -> 쓴다 -> 손 떼고 Enter      O 하나
    x + Enter  -> 쓴다 -> 손 떼고 Enter      X 하나
    n + Enter  -> 쓴다 -> 손 떼고 Enter      기타 (누르기, 직선, 아무렇게나)
    d + Enter                                방금 것 지우기
    q + Enter                                끝

    한 글자마다 csv 를 통째로 다시 쓴다. 중간에 꺼져도 그때까지는 남는다.
"""
import argparse
import datetime
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

SAMPLE_HZ = 100.0          # main_ifs 발행 주기와 같게
TOP_FACE_MAX_Z = -0.013    # ifs_algorithm.cpp 의 kSideFaceMinZ. cp z 가 이보다 작으면 윗면
MIN_GAP_SAMPLES = 5        # 이보다 짧게 떨어진 접촉은 한 획으로 본다 (0.05 s)
TARGET_PER_LABEL = 50
LABELS = {'o': 'O', 'x': 'X', 'n': 'N'}
EXPECTED_STROKES = {'O': 1, 'X': 2}

SENSORS = (1, 2)
SENSOR_FIELDS = ('cp_x', 'cp_y', 'cp_z', 'contact', 'mag',
                 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz',                    # wrench_corrected
                 'raw_Fx', 'raw_Fy', 'raw_Fz', 'raw_Mx', 'raw_My', 'raw_Mz')  # ftdata_tcp
COLUMNS = (['trial', 'label', 'face', 'writer', 't_sec']
           + [f's{s}_{f}' for s in SENSORS for f in SENSOR_FIELDS])


def wrench_list(m):
    w = m.wrench
    return [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]


class SensorState:
    def __init__(self):
        self.cp = (0.0, 0.0, 0.0)
        self.contact = False
        self.mag = float('nan')
        self.wrench = [float('nan')] * 6
        self.raw = [float('nan')] * 6
        self.cp_count = 0


class GestureRecorder(Node):
    def __init__(self):
        super().__init__('record_gesture')
        self.lock = threading.Lock()
        self.state = {s: SensorState() for s in SENSORS}
        self.recording = False
        self.samples = []
        self.t0 = 0.0
        for s in SENSORS:
            ns = f'/sensor{s}/contactsensing'
            st = self.state[s]
            self.create_subscription(
                PointStamped, ns + '/cp', lambda m, st=st: self._cp(st, m), 100)
            self.create_subscription(
                Bool, ns + '/contact_state', lambda m, st=st: self._contact(st, m), 100)
            self.create_subscription(
                Float64, ns + '/force_magnitude', lambda m, st=st: self._mag(st, m), 100)
            self.create_subscription(
                WrenchStamped, ns + '/wrench_corrected',
                lambda m, st=st: self._wrench(st, m), 100)
            self.create_subscription(
                WrenchStamped, f'/ur10skku/ftdata_tcp_{s}',
                lambda m, st=st: self._raw(st, m), 100)
        self.create_timer(1.0 / SAMPLE_HZ, self._tick)

    def _cp(self, st, m):
        st.cp = (m.point.x, m.point.y, m.point.z)
        st.cp_count += 1

    def _contact(self, st, m):
        st.contact = m.data

    def _mag(self, st, m):
        st.mag = m.data

    def _wrench(self, st, m):
        st.wrench = wrench_list(m)

    def _raw(self, st, m):
        st.raw = wrench_list(m)

    def _tick(self):
        with self.lock:
            if not self.recording:
                return
            row = [time.time() - self.t0]
            for s in SENSORS:
                st = self.state[s]
                row += [*st.cp, 1.0 if st.contact else 0.0, st.mag, *st.wrench, *st.raw]
            self.samples.append(row)

    def start(self):
        with self.lock:
            self.samples = []
            self.t0 = time.time()
            self.recording = True

    def stop(self):
        with self.lock:
            self.recording = False
            rows = self.samples
            self.samples = []
        return rows


def count_strokes(contact):
    """contact 0/1 열에서 획 수. MIN_GAP_SAMPLES 보다 짧게 떨어진 것은 한 획으로 본다."""
    strokes, gap, inside = 0, MIN_GAP_SAMPLES, False
    for c in contact:
        if c:
            if not inside and gap >= MIN_GAP_SAMPLES:
                strokes += 1
            inside = True
            gap = 0
        else:
            inside = False
            gap += 1
    return strokes


def summarize(rows, sensor):
    """한 센서 기준 요약: (접촉 샘플 수, 획 수, 최대 힘, 면, 가로 mm, 세로 mm)."""
    base = 1 + SENSORS.index(sensor) * len(SENSOR_FIELDS)
    contact = [r[base + 3] > 0.5 for r in rows]
    pts = [r[base:base + 3] for r, c in zip(rows, contact) if c]
    mags = [r[base + 4] for r, c in zip(rows, contact) if c and r[base + 4] == r[base + 4]]
    if not pts:
        return 0, 0, 0.0, '-', 0.0, 0.0
    top = sum(1 for p in pts if p[2] < TOP_FACE_MAX_Z)
    face = 'top' if top * 2 > len(pts) else 'side'
    # 윗면은 (x, y), 옆면은 (x, z) 평면에 써진다
    k = 1 if face == 'top' else 2
    us = [p[0] for p in pts]
    vs = [p[k] for p in pts]
    return (len(pts), count_strokes(contact), max(mags) if mags else 0.0, face,
            (max(us) - min(us)) * 1000.0, (max(vs) - min(vs)) * 1000.0)


def save(path, trials, face, writer):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as h:
        h.write(','.join(COLUMNS) + '\n')
        for trial_id, label, rows in trials:
            head = f'{trial_id},{label},{face},{writer},'
            for r in rows:
                h.write(head + ','.join(f'{v:.6f}' for v in r) + '\n')
    os.replace(tmp, path)


def counts_text(trials):
    n = {lab: 0 for lab in LABELS.values()}
    for _, label, _ in trials:
        n[label] += 1
    return (f'O {n["O"]}/{TARGET_PER_LABEL}   X {n["X"]}/{TARGET_PER_LABEL}   '
            f'기타 {n["N"]}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--face', required=True, choices=('top', 'side'),
                        help='이번에 쓸 면. 기록된 cp 가 다른 면이면 경고한다')
    parser.add_argument('--writer', default='unknown', help='쓴 사람')
    parser.add_argument('--out', default=None)
    args = parser.parse_args()

    writer = args.writer.replace(',', '_')
    out_path = args.out or os.path.expanduser('~/gesture_{}_{}.csv'.format(
        args.face, datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))

    rclpy.init()
    node = GestureRecorder()
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    time.sleep(2.0)
    for s in SENSORS:
        ok = node.state[s].cp_count > 0
        print(f'센서{s} cp: {"들어옴" if ok else "안 들어옴 (main_ifs 확인)"}')
    face_name = '윗면' if args.face == 'top' else '옆면'
    print(f'\n{face_name}에 쓰기.  저장: {out_path}')
    print('  o / x / n(기타) + Enter 로 시작 -> 쓰고 손 뗀 뒤 Enter')
    print('  d + Enter 방금 것 지우기,  q + Enter 끝\n')

    trials = []
    next_id = 1
    try:
        while True:
            cmd = input(f'[{counts_text(trials)}] > ').strip().lower()
            if cmd == 'q':
                break
            if cmd == 'd':
                if trials:
                    trial_id, label, _ = trials.pop()
                    save(out_path, trials, args.face, writer)
                    print(f'   #{trial_id} {label} 지움\n')
                else:
                    print('   지울 것이 없습니다\n')
                continue
            if cmd not in LABELS:
                print('   o / x / n / d / q 중 하나\n')
                continue

            label = LABELS[cmd]
            node.start()
            input(f'   #{next_id} {label} 기록 중... 다 쓰고 손 뗀 뒤 Enter > ')
            rows = node.stop()
            trials.append((next_id, label, rows))
            save(out_path, trials, args.face, writer)

            print(f'   {len(rows)} 샘플 ({len(rows) / SAMPLE_HZ:.1f} s)')
            touched = False
            for s in SENSORS:
                n, strokes, peak, face, w, h = summarize(rows, s)
                if n == 0:
                    print(f'   센서{s}: 접촉 없음')
                    continue
                touched = True
                notes = []
                if face != args.face:
                    notes.append(f'면이 {face} 로 잡힘')
                expected = EXPECTED_STROKES.get(label)
                if expected is not None and strokes != expected:
                    notes.append(f'{label} 는 보통 {expected}획')
                note = ('   <- ' + ', '.join(notes)) if notes else ''
                print(f'   센서{s}: 접촉 {n / SAMPLE_HZ:.2f} s, {strokes}획, '
                      f'최대 {peak:.1f} N, 크기 {w:.0f} x {h:.0f} mm{note}')
            if not touched:
                print('   접촉이 안 잡혔습니다. d 로 지우고 다시 쓰세요.')
            print()
            next_id += 1
    except (KeyboardInterrupt, EOFError):
        node.stop()

    if os.path.exists(out_path):
        print(f'\n저장: {out_path}   ({counts_text(trials)})')
    else:
        print('\n기록한 글자가 없어 저장하지 않았습니다.')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
