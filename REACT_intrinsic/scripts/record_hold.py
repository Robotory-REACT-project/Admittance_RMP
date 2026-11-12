#!/usr/bin/env python3
"""누른 채로 힘 방향 바꾸기 기록기. 한 자리를 누른 상태에서 힘의 방향(과 크기)을 바꾸며
그동안의 모든 순간을 남긴다.

record_points.py 는 누른 뒤 1.5 초의 중앙값 하나만 남겨서, 같은 자리에서 힘 방향에 따라
접촉점이 어떻게 변하는지는 볼 수 없다. 이 기록기는 자리 입력은 record_points.py 와 같고
(s / t / e), 기록은 매 순간(main_ifs 접촉점 주기, 약 100 Hz)을 전부 남긴다.
접촉점이 (0,0,0) 인 순간(못 찾은 순간)도 버리지 않는다.

쓰는 법:
    python3 record_hold.py                  # 센서 1
    python3 record_hold.py --sensor 2

    누를 자리 + Enter  -> 누르고, 누른 채로 힘 방향을 천천히 바꾼다 -> 손 떼고 Enter
        t 0 58       윗면  x=0mm, y=58mm
        e 0          모서리 가운데 x=0mm      e 0 2   모서리 단면 s=2mm
        s 0 -9       옆면  x=0mm, z=-9mm
    d + Enter        방금 기록 지우기
    q + Enter        끝

    끝이 둥근 단단한 막대(펜 뒤쪽 등)로 누르는 것을 권한다. 손가락은 각도를 바꿀 때 손끝이
    굴러 접촉점이 같이 움직인다.

한 자리 여러 번 누르기 (11/04):
    두 Enter 사이에 같은 자리를 몇 번이든 눌렀다 뗐다 해도 된다 (세기는 아무렇게나).
    기록은 원래부터 두 Enter 사이 모든 순간이고, 요약은 힘으로 누름을 나눠 누름마다 보여 준다
    (닿은 직후 0.1 s 오차, 힘이 붙은 뒤 오차, 최대 힘, 앞 누름과의 간격).
    python3 record_hold.py --sensor 2 --grid s2top    # 미리 넣은 자리를 Enter 만으로 차례대로
        자리 목록: 옆면 z 150 줄 x 0 / -30 / -60 / -90 / +30, 비교용 안쪽 (0, 70)
        Enter = 그 자리 기록 시작, 다시 Enter = 끝 -> 다음 자리.  번호 + Enter = 그 자리 다시 (앞 기록 지움)

남기는 것 (csv 한 줄 = 한 순간):
    trial, label, face, true_x/y/z [m]    누른 자리 (서포터 좌표)
    t_sec                                 기록 시작부터 [s]
    Fx..Mz                                F/T 원본 (/ur10skku/ftdata_N, 센서 좌표 그대로)
    mag                                   main_ifs force_magnitude [N]
    cp_x/y/z                              main_ifs 접촉점 (못 찾으면 0,0,0)
    cFx..cMz                              main_ifs wrench_corrected (배율 보정 후, 서포터 좌표)
원본이 있으니 지금 코드와 다른 보정식을 같은 데이터로 다시 계산할 수 있다.
"""
import argparse
import datetime
import math
import os
import statistics
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import record_points as rp  # noqa: E402  자리 입력·모서리 좌표·오차 계산을 그대로 쓴다

PRESS_ON = rp.PRESS_ON      # 이 힘[N] 이상인 순간만 요약에 넣는다 (기록은 전부 남긴다)
RELEASE_N = 1.5             # 누름 나누기: 이 힘 아래로 RELEASE_SEC 이상 내려가면 뗀 것으로 본다
RELEASE_SEC = 0.15
ONSET_SEC = 0.10            # '닿은 직후' = PRESS_ON 을 넘고 이 시간 동안

# 미리 넣은 자리 목록 (--grid). 11/04: 센서2 옆면 위쪽 처음 누름이 낮게 뜨는 현상 확인용
HOLD_GRIDS = {
    's2top': ['s 0 150', 's -30 150', 's -60 150', 's -90 150', 's 30 150', 's 0 70'],
}
STALE_SEC = 0.2             # F/T 원본이 이보다 오래되면 그 순간은 쓰지 않고 '끊김' 으로 센다

HEADER = ('trial,label,face,true_x,true_y,true_z,t_sec,'
          'Fx,Fy,Fz,Mx,My,Mz,mag,cp_x,cp_y,cp_z,cFx,cFy,cFz,cMx,cMy,cMz')


def target_point(target):
    """record_points.parse_target 결과 -> 누른 자리 (x, y, z) [m] (서포터 좌표)."""
    _, _, ib, _, _, a, b = target
    if ib is None:                      # 모서리: b = 단면 s
        if b < 0.0:
            return (a, rp.EDGE_CENTER_Y + b, -0.019)
        if b > rp.EDGE_ARC:
            return (a, 0.068, rp.EDGE_CENTER_Z + (b - rp.EDGE_ARC))
        ang = b / rp.EDGE_RADIUS
        return (a, rp.EDGE_CENTER_Y + rp.EDGE_RADIUS * math.sin(ang),
                rp.EDGE_CENTER_Z - rp.EDGE_RADIUS * math.cos(ang))
    if ib == 1:                         # 윗면: b = y
        return (a, b, -0.019)
    return (a, 0.068, b)                # 옆면: b = z


def force_angle_deg(raw):
    """F/T 원본(센서 좌표) -> 힘 방향 [도]. 0 = 윗면을 위에서 수직, 90 = 옆면을 옆에서 수직.

    센서 -> 서포터 좌표는 (x, y, z) -> (-x, y, -z) (launch 의 wrench calibration TF).
    """
    fy_s, fz_s = raw[1], -raw[2]
    return math.degrees(math.atan2(-fy_s, fz_s))


class HoldRecorder(Node):
    def __init__(self, sensor, ft_topic):
        super().__init__('record_hold')
        self.lock = threading.Lock()
        self.ft = None
        self.ft_time = 0.0
        self.mag = float('nan')
        self.corr = [float('nan')] * 6
        self.seen = {'ft': False, 'mag': False, 'cp': False}
        self.rows = None            # 기록 중이면 list
        self.t0 = 0.0
        self.stale = 0
        ns = f'/sensor{sensor}/contactsensing'
        self.create_subscription(WrenchStamped, ft_topic, self._ft, 100)
        self.create_subscription(Float64, ns + '/force_magnitude', self._mag, 100)
        self.create_subscription(WrenchStamped, ns + '/wrench_corrected', self._corr, 100)
        self.create_subscription(PointStamped, ns + '/cp', self._cp, 100)

    def _ft(self, m):
        w = m.wrench
        self.ft = (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z)
        self.ft_time = time.time()
        self.seen['ft'] = True

    def _mag(self, m):
        self.mag = m.data
        self.seen['mag'] = True

    def _corr(self, m):
        w = m.wrench
        self.corr = [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]

    def _cp(self, m):
        # 접촉점 메시지(약 100 Hz)를 시계로 쓴다. 그 순간의 최신 F/T 원본을 같이 남긴다.
        self.seen['cp'] = True
        with self.lock:
            if self.rows is None:
                return
            now = time.time()
            if self.ft is None or now - self.ft_time > STALE_SEC:
                self.stale += 1
                return
            p = (m.point.x, m.point.y, m.point.z)
            self.rows.append((now - self.t0, *self.ft, self.mag, *p, *self.corr))

    def start(self):
        with self.lock:
            self.rows = []
            self.stale = 0
            self.t0 = time.time()

    def stop(self):
        with self.lock:
            rows, self.rows = self.rows, None
        return rows


def split_presses(rows):
    """한 기록 -> 누름마다 [시작 index, 끝 index). force_magnitude 로 나눈다."""
    presses, start, below = [], None, None
    for i, r in enumerate(rows):
        mag = r[7]
        if mag != mag:
            continue
        if start is None:
            if mag >= PRESS_ON:
                start, below = i, None
            continue
        if mag < RELEASE_N:
            if below is None:
                below = i
            elif r[0] - rows[below][0] >= RELEASE_SEC:
                presses.append((start, below)); start, below = None, None
        else:
            below = None
    if start is not None:
        presses.append((start, below if below is not None else len(rows)))
    return presses


def press_table(rows, target):
    """누름마다 한 줄: 간격, 길이, 최대 힘, 닿은 직후 / 힘 붙은 뒤 접촉점 오차, 못 찾음."""
    if target is None:
        return []
    _, _, _, na, nb, _, _ = target
    out = [f'   누름  간격[s]  길이[s]  최대힘[N] |  닿은 직후 0.1 s  {na},{nb} [mm] |  힘 붙은 뒤 (최대의 70% 이상)  {na},{nb} [mm] | 못 찾음']
    prev_end = None
    for k, (a, b) in enumerate(split_presses(rows), start=1):
        seg = rows[a:b]
        if len(seg) < 5:
            continue
        mags = [r[7] for r in seg]; peak = max(mags)
        t0 = seg[0][0]
        found = lambda rs: [r for r in rs if abs(r[8]) + abs(r[9]) + abs(r[10]) > 1e-9]
        onset = found([r for r in seg if r[0] - t0 <= ONSET_SEC])
        plat = found([r for r in seg if r[7] >= 0.7 * peak])
        def err(rs):
            if not rs:
                return '      없음     '
            e = [rp.target_error(target, r[8:11]) for r in rs]
            return f'{statistics.median([v[0] for v in e]):+6.1f},{statistics.median([v[1] for v in e]):+6.1f}'
        miss = 100.0 * (1 - len(found(seg)) / len(seg))
        gap = f'{t0 - prev_end:6.1f}' if prev_end is not None else '     -'
        out.append(f'   {k:3d}  {gap}  {seg[-1][0] - t0:6.1f}  {peak:8.1f}  |  {err(onset):>15}  |  {err(plat):>15}  | {miss:4.0f}%')
        prev_end = seg[-1][0]
    return out


def summarize(rows, target):
    """한 번 기록한 것 -> 화면에 찍을 요약 문자열들."""
    on = [r for r in rows if r[7] == r[7] and r[7] >= PRESS_ON]
    if not on:
        return [f'   {PRESS_ON:.0f} N 넘은 순간이 없습니다 (기록 {len(rows)}줄은 남김).']
    ang = [force_angle_deg(r[1:7]) for r in on]
    mag = [r[7] for r in on]
    found = [r for r in on if abs(r[8]) + abs(r[9]) + abs(r[10]) > 1e-9]
    lines = [
        f'   누른 시간 {on[-1][0] - on[0][0]:4.1f} s ({len(on)} 순간),  '
        f'힘 {min(mag):4.1f} ~ {max(mag):4.1f} N,  '
        f'힘 방향 {min(ang):+5.0f}° ~ {max(ang):+5.0f}°  (0 = 위에서, 90 = 옆에서)',
        f'   접촉점 못 찾음 {100.0 * (1 - len(found) / len(on)):4.1f}%']
    if found and target is not None:
        _, _, _, na, nb, _, _ = target
        errs = [rp.target_error(target, r[8:11]) for r in found]
        ea = [e[0] for e in errs]
        eb = [e[1] for e in errs]
        lines.append(
            f'   찾은 접촉점 오차 중앙  {na} {statistics.median(ea):+6.1f} mm  '
            f'{nb} {statistics.median(eb):+6.1f} mm   '
            f'({nb} 범위 {min(eb):+6.1f} ~ {max(eb):+6.1f} mm)')
    if len(split_presses(rows)) >= 1:
        lines += press_table(rows, target)
    return lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor', type=int, default=1, choices=(1, 2))
    parser.add_argument('--ft-topic', default=None, help='F/T 원본 토픽 (기본 /ur10skku/ftdata_N)')
    parser.add_argument('--out', default=None)
    parser.add_argument('--grid', default=None, choices=sorted(HOLD_GRIDS),
                        help='미리 넣은 자리를 Enter 만으로 차례대로 (한 자리에서 여러 번 눌러도 됨)')
    args = parser.parse_args()
    out_path = args.out or os.path.expanduser('~/hold_{}.csv'.format(
        datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
    ft_topic = args.ft_topic or f'/ur10skku/ftdata_{args.sensor}'

    rclpy.init()
    node = HoldRecorder(args.sensor, ft_topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    time.sleep(2.0)
    print(f'센서 {args.sensor} 누른 채 방향 바꾸기 기록기.  F/T 원본: {ft_topic}')
    for key, name in (('ft', 'F/T 원본'), ('mag', 'force_magnitude'), ('cp', '접촉점 cp')):
        if not node.seen[key]:
            print(f'!! {name} 가 안 들어옵니다. FTGetMain / main_ifs 와 이 터미널의 '
                  f'ROS_DOMAIN_ID (지금 {os.environ.get("ROS_DOMAIN_ID", "없음")}) 를 확인하세요.')
    print('  예)  t 0 58   윗면 x=0 y=58     e 0   모서리 가운데     e 0 2   모서리 s=2mm'
          '     s 0 -9   옆면 x=0 z=-9')
    print('  자리 + Enter -> 누르기 (힘 방향을 바꾸거나, 여러 번 눌렀다 떼도 됨) -> 손 떼고 Enter.'
          '   d = 방금 것 지우기, q = 끝\n')

    trials = []     # (label, face, true point, rows)
    grid = HOLD_GRIDS[args.grid] if args.grid else None
    nxt = 0         # 격자에서 Enter 로 잴 차례 (0부터)
    done = {}       # 격자 번호 -> trials 안의 위치
    if grid:
        print(f'  격자 {args.grid}: ' + '  '.join(f'{i + 1}) {rp.describe(g)}' for i, g in enumerate(grid)))
        print('  Enter = 그 자리 기록 시작 -> 같은 자리를 세기 바꿔 여러 번 눌렀다 떼기 -> Enter = 끝.'
              '  번호 + Enter = 그 자리 다시.\n')
    try:
        while True:
            number = None
            if grid and nxt < len(grid):
                prompt = f'[{nxt + 1}/{len(grid)}] {rp.describe(grid[nxt])}  (Enter=시작, 번호=그 자리, d=지우기, q=끝) > '
            elif grid:
                prompt = '격자 끝. 번호=다시 재기, 자리 직접 입력, q=끝 > '
            else:
                prompt = f'[{len(trials) + 1}] 누를 자리 (d=지우기, q=끝) > '
            entry = input(prompt).strip()
            if entry.lower() == 'q':
                break
            if entry.lower() == 'd':
                if trials:
                    print(f'   지움: {trials.pop()[0]}\n')
                    done = {k: v for k, v in done.items() if v < len(trials)}
                continue
            if grid and not entry:
                if nxt >= len(grid):
                    continue
                number = nxt + 1
            elif grid and entry.isdigit() and 1 <= int(entry) <= len(grid):
                number = int(entry)
            if number is not None:
                entry = grid[number - 1]
                print(f'   {number}번  {rp.describe(entry)}')
            if not entry:
                continue
            target = rp.parse_target(entry)
            if target is None:
                print('   못 읽었습니다. 예: t 0 58  /  e 0  /  e 0 2  /  s 0 -9\n')
                continue
            node.start()
            input('   기록 중... 누르고 (여러 번 눌러도 됨), 손 떼고 Enter ')
            rows = node.stop()
            print(f'   {len(rows)} 줄 기록 (F/T 끊김 {node.stale} 순간)')
            for line in summarize(rows, target):
                print(line)
            print()
            if number is not None and number in done:
                # 같은 자리를 다시 재면 앞 기록을 바꾼다 (csv 에 같은 자리가 두 번 남지 않게)
                trials[done[number]] = (entry, target[0], target_point(target), rows)
                print(f'   {number}번 앞 기록을 이번 기록으로 바꿨습니다.\n')
            else:
                trials.append((entry, target[0], target_point(target), rows))
                if number is not None:
                    done[number] = len(trials) - 1
            if number is not None and number > nxt:
                nxt = number
    except (KeyboardInterrupt, EOFError):
        pass

    with open(out_path, 'w', encoding='utf-8') as handle:
        handle.write(HEADER + '\n')
        for number, (label, face, truth, rows) in enumerate(trials, start=1):
            head = f'{number},{label},{face},' + ','.join(f'{v:.5f}' for v in truth)
            for r in rows:
                handle.write(head + ',' + ','.join(f'{v:.6f}' for v in r) + '\n')
    print(f'\n저장: {out_path}  ({len(trials)} 번, {sum(len(t[3]) for t in trials)} 줄)')

    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
