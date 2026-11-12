#!/usr/bin/env python3
"""접촉점 기록기. 서포터의 각 지점을 누르면서 F/T 원본과 계산된 접촉점을 함께 모은다.

F/T 6축 원본이 있어야 어느 채널이 얼마나 어긋났는지 계산할 수 있다.
계산된 접촉점은 그 진단이 맞는지 대조하는 용도다.

누른 자리를 실제 좌표로 같이 넣으면 오차를 바로 찍어주고 csv 로도 남긴다.

쓰는 법:
    python3 record_points.py            # 센서 1
    python3 record_points.py --sensor 2

    누를 자리를 넣고 Enter -> 그 다음 누르면 -> 힘이 올라온 뒤부터 평균이 기록된다.
        s -50 60     옆면(법선 +y)  가로 x=-50mm, 세로 z=+60mm
        t -50 20     윗면(법선 -z)  가로 x=-50mm, 세로 y=+20mm
        e -50        모서리(윗면-옆면 사이 둥근 부분 가운데)  가로 x=-50mm
        e -50 2      모서리 단면을 따라 2mm 옆면 쪽 (아래 '모서리' 설명)
    좌표를 모르면 예전처럼 번호만 넣어도 된다 (그러면 오차는 못 낸다).
    q 를 넣으면 표를 출력하고 끝난다.

격자 (좌표를 안 쳐도 된다):
    python3 record_points.py --sensor 2 --grid side7
    python3 record_points.py --sensor 2 --grid sideedge   # 옆면 바깥 띠 36자리
    프롬프트에 다음 자리가 떠 있다. Enter -> 누르기 -> 떼기 -> Enter ... 만 되풀이하면 된다.
    누르기를 기다리는 동안 Enter 를 치면 그 자리를 건너뛰고 바로 다음 자리로 간다 (11/03).
    안 눌러서 시간이 지나도 다음 자리로 넘어간다. 건너뛴 자리는 끝에 번호로 알려 준다.
    번호를 치면 그 자리를 다시 잰다. 앞 번호를 다시 재도 Enter 는 하던 차례로 돌아간다.
    접촉점이 안 나와도 F/T 원본 렌치는 항상 기록한다 (csv 의 cp 는 nan).
"""
import argparse
import datetime
import math
import os
import select
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float64

SECONDS = 1.5          # 한 지점당 평균 낼 시간
PRESS_ON = 3.0         # 이 힘[N]을 넘으면 누른 것으로 보고 평균을 시작한다
WAIT_SEC = 15.0        # Enter 후 이 시간 안에 안 누르면 건너뛴다

# JSshape 의 큰 평면 두 개. 면 위에서 자리를 정하는 두 축만 쓴다.
# (ifs_algorithm.cpp 의 kSideFaceMinZ 와 같은 기준으로 나눈 면이다)
FACES = {
    's': ('옆면', 0, 2, 'x', 'z'),   # 법선 +y : 가로 x, 세로 z
    't': ('윗면', 0, 1, 'x', 'y'),   # 법선 -z : 가로 x, 세로 y
    'e': ('모서리', 0, None, 'x', 's'),   # 윗면-옆면 사이 둥근 부분 : 가로 x, 단면 s
}

# 모서리 = 윗면(z -19)과 옆면(y 68) 사이 반지름 6 mm 둥근 부분.
# JSshape_surface.stl 에서 중심 (y 62, z -13), 꼭짓점 6개가 모두 중심에서 6.0 mm.
# 단면 좌표 s [m] 는 모서리 단면을 따라 잰 길이다.
#   s < 0                 윗면 (y = 62 mm 에서 안쪽으로 |s|)
#   0 <= s <= 9.42 mm     둥근 부분 (s = 0 윗면 쪽 끝, 4.71 mm 가운데, 9.42 mm 옆면 쪽 끝)
#   s > 9.42 mm           옆면 (z = -13 mm 에서 아래로 s - 9.42)
# 'e x' 만 치면 가운데(s = 4.71 mm), 'e x s' 로 s 를 mm 로 줄 수도 있다 (s 는 둥근 부분 시작 기준).
# 오차 s 는 + 면 접촉점이 옆면 쪽으로, - 면 윗면 쪽으로 밀렸다는 뜻이다.
EDGE_CENTER_Y = 0.062
EDGE_CENTER_Z = -0.013
EDGE_RADIUS = 0.006
EDGE_ARC = EDGE_RADIUS * math.pi / 2.0


def edge_s(point):
    """서포터 좌표 (x, y, z) [m] -> 모서리 단면 좌표 s [m]."""
    y, z = point[1], point[2]
    if y <= EDGE_CENTER_Y:
        return y - EDGE_CENTER_Y
    if z >= EDGE_CENTER_Z:
        return EDGE_ARC + (z - EDGE_CENTER_Z)
    return EDGE_RADIUS * math.atan2(y - EDGE_CENTER_Y, EDGE_CENTER_Z - z)


def target_error(target, point):
    """(면 첫째 축 오차, 둘째 축 오차) [mm]. 모서리의 둘째 축은 단면 좌표 s."""
    _, ia, ib, _, _, ta, tb = target
    ea = (point[ia] - ta) * 1000.0
    second = edge_s(point) if ib is None else point[ib]
    return ea, (second - tb) * 1000.0


# 미리 넣어둔 격자. 번호만 쳐도 되도록 한다.
# 순서는 윗면 모서리 쪽 줄부터 아래로, 각 줄은 왼쪽에서 오른쪽으로.
GRIDS = {
    # 4x4 격자(1~16) + 면 끝 8자리(17~24). 지금 쓰는 격자.
    # 7x7 은 안쪽만 재는 바람에 모서리가 외삽 구간으로 남았다. 자리 수를
    # 줄이더라도 면 끝을 실제로 재는 쪽이 보정식에 낫다.
    'side4e': ('s', [-90, -30, 30, 90], [-5, 45, 95, 145],
               [(-100, -13), (0, -13), (100, -13),
                (-100, 70), (100, 70),
                (-100, 154), (0, 154), (100, 154)]),
    'side7': ('s', [-90, -60, -30, 0, 30, 60, 90], [-5, 20, 45, 70, 95, 120, 145]),
    'side5': ('s', [-80, -40, 0, 40, 80], [10, 40, 70, 100, 130]),
    'top5': ('t', [-80, -40, 0, 40, 80], [45, 15, -15, -45]),
    'top7': ('t', [-90, -60, -30, 0, 30, 60, 90], [45, 15, -15, -45]),
    # 옆면 바깥 띠 (11/03, 사용자 지정 36자리). 옆면은 x -100 ~ 100, z -13 (아랫면 쪽 둥근 부분 끝) ~ 155 mm.
    # 1 ~ 16   네 귀퉁이: x -100 / -80 / 80 / 100  x  z -10 / 0 / 140 / 150
    # 17 ~ 26  위아래 끝: x -60 ~ 60 (30 간격)  x  z 150, -10
    # 27 ~ 36  양옆 끝:   x -100, 100  x  z 20 ~ 120 (25 간격)
    'sideedge': ('s', [-100, -80, 80, 100], [-10, 0, 140, 150],
                 [(-60, 150), (-30, 150), (0, 150), (30, 150), (60, 150),
                  (-60, -10), (-30, -10), (0, -10), (30, -10), (60, -10),
                  (-100, 20), (-100, 45), (-100, 70), (-100, 95), (-100, 120),
                  (100, 20), (100, 45), (100, 70), (100, 95), (100, 120)]),
}


def build_grid(name):
    """격자 이름 -> [(번호, 'face a b'), ...]

    격자를 다 채운 뒤, 정의에 네 번째 항목이 있으면 그 자리들을 뒤에 붙인다.
    면 끝처럼 격자에 안 맞는 자리를 따로 재려고 둔 것이다.
    """
    entry = GRIDS[name]
    face, xs, bs = entry[0], entry[1], entry[2]
    extras = entry[3] if len(entry) > 3 else []
    out = []
    for b in bs:
        for x in xs:
            out.append((len(out) + 1, f'{face} {x} {b}'))
    for x, b in extras:
        out.append((len(out) + 1, f'{face} {x} {b}'))
    return out


def print_grid(name, grid):
    """번호 배치를 표로 찍어 종이에 옮겨 적기 쉽게 한다.

    칸 위에 가로 좌표, 줄 앞에 세로 좌표 [mm] 를 붙인다. 표의 위아래·좌우가 커버를
    바깥에서 본 모양과 같다는 보장은 없으므로 (센서마다 다르다) 좌표로 맞출 것.
    격자 밖 자리는 표에 끼워 넣으면 위치가 거짓말이 되므로 좌표를 하나씩 풀어 쓴다.
    """
    face, xs, bs = GRIDS[name][:3]
    face_name, _, _, na, nb = FACES[face]
    cols = len(xs)
    grid_count = cols * len(bs)
    print(f'   격자 번호 배치 — {face_name}, 칸 = {na} [mm], 줄 = {nb} [mm]')
    print(f'       {"":>1} {na:>5} ' + ''.join(f'{x:>5}' for x in xs))
    for row, b in enumerate(bs):
        numbers = grid[row * cols:(row + 1) * cols]
        print(f'       {nb} {b:>5} ' + ''.join(f'{n:>5}' for n, _ in numbers))
    if grid_count < len(grid):
        print('   격자 밖 자리 (면 끝)')
        for number, label in grid[grid_count:]:
            parts = label.split()
            print(f'     {number:>3}  {na}={parts[1]:>5}  {nb}={parts[2]:>5}')


def parse_target(text):
    """'s -50 60' -> ('옆면', 0, 2, 'x', 'z', -0.050, 0.060). 못 읽으면 None.

    모서리는 'e -50' (가운데) 또는 'e -50 2' (단면 s = 2 mm).
    """
    parts = text.split()
    if not parts or parts[0].lower() not in FACES:
        return None
    face = parts[0].lower()
    if face == 'e' and len(parts) == 2:
        parts = parts + [str(EDGE_ARC / 2.0 * 1000.0)]
    if len(parts) != 3:
        return None
    name, ia, ib, na, nb = FACES[face]
    try:
        a, b = float(parts[1]) / 1000.0, float(parts[2]) / 1000.0
    except ValueError:
        return None
    return name, ia, ib, na, nb, a, b


def describe(label):
    """'s -90 -5' -> '옆면 x -90 z -5'. 못 읽으면 그대로."""
    target = parse_target(label)
    if target is None:
        return label
    name, _, _, na, nb, a, b = target
    return f'{name} {na} {a * 1000.0:g} {nb} {b * 1000.0:g}'


class Recorder(Node):
    def __init__(self, sensor, ft_topic):
        super().__init__('record_points')
        self.cp = []
        self.mag = []
        self.ft = []
        self.create_subscription(
            PointStamped, f'/sensor{sensor}/contactsensing/cp', self._cp, 50)
        self.create_subscription(
            Float64, f'/sensor{sensor}/contactsensing/force_magnitude', self._mag, 50)
        self.create_subscription(WrenchStamped, ft_topic, self._ft, 50)

    def _cp(self, m):
        self.cp.append((m.point.x, m.point.y, m.point.z))

    def _mag(self, m):
        self.mag.append(m.data)

    def _ft(self, m):
        w = m.wrench
        self.ft.append((
            w.force.x, w.force.y, w.force.z,
            w.torque.x, w.torque.y, w.torque.z))

    def wait_for_press(self, timeout=WAIT_SEC):
        """누를 때까지 기다린다. 닿으면 'press', 시간 초과면 'timeout', Enter 를 치면 'skip'.

        Enter 직후부터 평균을 내면 아직 안 누른 구간이 섞여 F/T 중앙값이
        깎인다. 그래서 힘이 올라온 뒤에 평균 구간을 시작한다.
        """
        self.mag.clear()
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.mag and self.mag[-1] >= PRESS_ON:
                return 'press'
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                return 'skip'
        return 'timeout'

    def sample(self):
        self.cp.clear()
        self.mag.clear()
        self.ft.clear()
        end = time.time() + SECONDS
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        # 접촉이 없을 때 발행되는 (0,0,0) 은 버린다
        pts = [p for p in self.cp if abs(p[0]) + abs(p[1]) + abs(p[2]) > 1e-9]
        raw = [statistics.median(v[i] for v in self.ft) for i in range(6)] \
            if self.ft else [float('nan')] * 6
        weakest = min(self.mag) if self.mag else float('nan')
        if not pts:
            return None, raw, 0, weakest
        mid = [statistics.median(v[i] for v in pts) for i in range(3)]
        return mid, raw, len(pts), weakest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor', type=int, default=1, choices=(1, 2))
    parser.add_argument('--ft-topic', default=None,
                        help='기본값: /ur10skku/ftdata_<센서번호>')
    parser.add_argument('--out', default=None,
                        help='기본값: ~/contact_points_s<센서>[_<격자>]_<날짜시각>.txt')
    parser.add_argument('--grid', default=None, choices=sorted(GRIDS),
                        help='미리 넣어둔 격자를 쓰면 번호만 입력하면 된다')
    args = parser.parse_args()

    # 기본 이름에 센서와 격자를 넣어, 나중에 파일만 보고 어느 기록인지 알 수 있게 한다.
    out_path = args.out or os.path.expanduser(
        '~/contact_points_s{}{}_{}.txt'.format(
            args.sensor, f'_{args.grid}' if args.grid else '',
            datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))

    ft_topic = args.ft_topic or f'/ur10skku/ftdata_{args.sensor}'
    rclpy.init()
    node = Recorder(args.sensor, ft_topic)
    print(f'F/T 원본 토픽: {ft_topic}')
    rows = []
    print(f'센서 {args.sensor} 기록기. 누를 자리를 넣고 Enter -> 누르기 -> {SECONDS} 초 유지.')
    grid = build_grid(args.grid) if args.grid else None
    nxt = 0    # 격자 차례. Enter 를 치면 nxt + 1 번을 잰다
    if grid:
        print(f'  격자 {args.grid} — 프롬프트에 뜬 자리를 Enter 로 잰다. 좌표는 안 쳐도 된다.')
        print('  번호를 치면 그 자리를 다시 잰다 (앞 번호를 다시 재도 Enter 는 하던 차례로).')
        print_grid(args.grid, grid)
    else:
        print('  예)  s -50 60   옆면 x=-50mm z=+60mm      t -50 20   윗면 x=-50mm y=+20mm')
        print('       e -50      모서리 가운데 x=-50mm    (오차 s: + 옆면 쪽, - 윗면 쪽으로 밀림)')
    print('  좌표를 직접 쳐도 된다.   q + Enter 로 종료.')
    print(f'  Enter 친 뒤에 누르면 됩니다 — {PRESS_ON:.0f} N 을 넘는 순간부터 재기 시작합니다.\n')
    try:
        while True:
            if grid and nxt < len(grid):
                prompt = (f'[{nxt + 1}/{len(grid)}] {describe(grid[nxt][1])}'
                          f'   (Enter=재기, 번호=그 자리, q=종료) > ')
            elif grid:
                prompt = '격자를 다 돌았습니다. 다시 잴 번호, 또는 q=종료 > '
            else:
                prompt = '누를 자리 (q=종료) > '
            entry = input(prompt).strip()
            if entry.lower() == 'q':
                break
            number = None
            if grid:
                if not entry:                      # Enter 만 -> 차례인 번호
                    if nxt >= len(grid):
                        continue
                    number = nxt + 1
                elif entry.isdigit() and 1 <= int(entry) <= len(grid):
                    number = int(entry)
                    print(f'   {number}번 = {describe(grid[number - 1][1])}')
                if number is not None:
                    label = grid[number - 1][1]
                else:
                    label = entry
            else:
                if not entry:
                    continue
                label = entry
            target = parse_target(label)
            if target is None and len(label.split()) > 1:
                print('   못 읽었습니다. 예: s -50 60  /  t -50 20\n')
                continue
            skip_hint = ', Enter = 건너뛰기' if number is not None else ''
            print(f'   누르세요... ({PRESS_ON:.0f} N 넘으면 시작{skip_hint})', end='', flush=True)
            state = node.wait_for_press()
            if state != 'press':
                why = 'Enter 로 건너뜀' if state == 'skip' else f'{WAIT_SEC:.0f} 초 안에 안 누름'
                if number is not None:
                    # 격자에서는 바로 다음 자리로 넘어간다. 건너뛴 자리는 끝에 알려 준다
                    if number > nxt:
                        nxt = number
                    print(f'\r   {number}번 {why} — 기록 안 함. 다시 재려면 {number} + Enter        \n')
                else:
                    print(f'\r   {why} — 기록하지 않았습니다.        \n')
                continue
            print(f'\r   감지됨. {SECONDS} 초 그대로 유지하세요.   ', end='', flush=True)
            point, raw, n, weakest = node.sample()
            print('\r' + ' ' * 44 + '\r', end='')
            if number is not None and number > nxt:
                nxt = number                       # 차례이거나 앞으로 건너뛴 번호면 그 다음부터
            if point is None:
                point = [float('nan')] * 3
                print('   접촉점은 안 잡혔지만 F/T 원본은 기록합니다.')
            rows.append((label, target, point, raw, n))
            print(f'   F/T  힘 [{raw[0]:7.3f} {raw[1]:7.3f} {raw[2]:7.3f}] N  '
                  f'모멘트 [{raw[3]:7.4f} {raw[4]:7.4f} {raw[5]:7.4f}] Nm')
            print(f'   접촉점 [{point[0]:8.4f} {point[1]:8.4f} {point[2]:8.4f}] m  (표본 {n})')
            if weakest < PRESS_ON:
                redo = f' ({number} + Enter)' if number is not None else ''
                print(f'   [주의] 재는 중에 힘이 {weakest:.1f} N 까지 빠졌습니다. '
                      f'다시 재는 게 좋습니다{redo}.')
            if target is not None:
                name, _, _, na, nb, _, _ = target
                ea, eb = target_error(target, point)
                print(f'   {name} 오차  {na} {ea:+7.1f} mm   {nb} {eb:+7.1f} mm'
                      f'   (거리 {(ea * ea + eb * eb) ** 0.5:6.1f} mm)')
            print()
    except (KeyboardInterrupt, EOFError):
        pass

    if grid:
        measured = {label for label, *_ in rows}
        missing = [(n, label) for n, label in grid if label not in measured]
        if missing:
            print('\n안 잰 자리: ' + ', '.join(f'{n}({describe(label)})' for n, label in missing))

    lines = []
    lines.append('===== 아래를 그대로 복사해서 보내세요 =====')
    lines.append(f'센서 {args.sensor}   (F/T 원본 = 센서 좌표계 그대로, 길이 m, 오차 mm)')
    lines.append(f'{"자리":>11} {"Fx":>8} {"Fy":>8} {"Fz":>8} {"Mx":>9} {"My":>9} {"Mz":>9}'
                 f'  |  {"cp_x":>8} {"cp_y":>8} {"cp_z":>8}  |  {"오차a":>7} {"오차b":>7}')
    for label, target, point, raw, _ in rows:
        if target is None:
            err = f'{"-":>7} {"-":>7}'
        else:
            ea, eb = target_error(target, point)
            err = f'{ea:+7.1f} {eb:+7.1f}'
        lines.append(f'{label:>11} {raw[0]:8.3f} {raw[1]:8.3f} {raw[2]:8.3f} '
                     f'{raw[3]:9.4f} {raw[4]:9.4f} {raw[5]:9.4f}  |  '
                     f'{point[0]:8.4f} {point[1]:8.4f} {point[2]:8.4f}  |  {err}')
    lines.append('==========================================')

    text = '\n'.join(lines)
    print('\n' + text)
    with open(out_path, 'w', encoding='utf-8') as handle:
        handle.write(text + '\n')

    # 보정식을 맞출 때 쓰기 좋게 csv 로도 남긴다.
    csv_path = os.path.splitext(out_path)[0] + '.csv'
    with open(csv_path, 'w', encoding='utf-8') as handle:
        handle.write('label,face,true_a_m,true_b_m,axis_a,axis_b,'
                     'Fx,Fy,Fz,Mx,My,Mz,cp_x,cp_y,cp_z,err_a_mm,err_b_mm,samples\n')
        for label, target, point, raw, n in rows:
            if target is None:
                face, ta, tb, na, nb, ea, eb = '', '', '', '', '', '', ''
            else:
                face, _, _, na, nb, ta, tb = target
                ea, eb = (f'{v:.2f}' for v in target_error(target, point))
            handle.write(
                f'{label},{face},{ta},{tb},{na},{nb},'
                + ','.join(f'{v:.5f}' for v in raw) + ','
                + ','.join(f'{v:.5f}' for v in point)
                + f',{ea},{eb},{n}\n')

    print(f'\n파일로도 저장했습니다: {out_path}')
    print(f'보정식용 csv:        {csv_path}')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
