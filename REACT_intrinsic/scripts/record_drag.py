#!/usr/bin/env python3
"""긋기 기록기. 자를 대고 직선으로 긋는 동안 접촉점과 F/T 를 통째로 남긴다.

누름 격자(record_points.py)로는 안 보이는 것들이 있다. 보정 지도가 접히거나
면 끝에서 포화하는 것은 그어봐야 드러난다.

직선을 그으면 '어디를 지나는지'는 몰라도 '직선이어야 한다'는 것은 안다.
그래서 기록된 점들이 직선에서 얼마나 벗어나는지로 오차를 잴 수 있다.

쓰는 법:
    python3 record_drag.py                  # 센서 1
    python3 record_drag.py --sensor 2

    이름 치고 Enter  -> 긋는다 -> Enter 로 종료   (한 번에 한 줄)
    q + Enter        -> csv 저장하고 끝

    이름은 자를 댄 자리를 적는다:
        h40    옆면 가로로 그음, 자를 z = 40 mm 에 댐
        v-50   옆면 세로로 그음, 자를 x = -50 mm 에 댐
        th15   윗면 가로로 그음, 자를 y = 15 mm 에 댐
        tv-40  윗면 세로로 그음, 자를 x = -40 mm 에 댐

격자 (줄 이름을 안 쳐도 된다):
    python3 record_drag.py --sensor 2 --grid side7      # 가로 7줄 + 세로 7줄 = 14줄
    python3 record_drag.py --sensor 2 --grid top5       # 가로 4줄 + 세로 5줄 = 9줄
    python3 record_drag.py --sensor 2 --grid top7       # 가로 4줄 + 세로 7줄 = 11줄
    python3 record_drag.py --sensor 2 --grid sideedge   # 옆면 바깥 띠, 18줄 x 양방향 = 36획
    record_points.py 의 같은 이름 격자에서 각 줄·각 칸을 따라 긋는다 (면 끝 자리는 뺌).
    프롬프트에 다음 줄이 떠 있다. Enter -> 긋기 -> Enter ... 만 되풀이하면 된다.
    Enter 를 치면 언제나 바로 다음 번호로 넘어간다 (11/03. 예전엔 점이 적으면 차례가 안 넘어갔다).
    번호를 치면 그 줄을 긋는다 (이미 그은 줄이면 앞 기록은 지운다).
    원본 렌치는 버리지 않는다: 접촉점이 나온 순간은 본 csv, 안 나온 순간은 <이름>_안나옴.csv.
    (예전엔 접촉점이 5개 미만이면 그 줄의 안나옴 기록까지 지웠다.)
    한 순간도 안 눌린 줄만 '안 그은 줄' 로 끝에 알려 준다.

방향 (11/03):
    줄 이름 끝에 긋는 방향을 붙일 수 있다.  h150+x = z 150 줄을 x 가 커지는 쪽으로,
    v-95-z = x -95 줄을 z 가 작아지는 쪽으로. 붙이면 다 그은 뒤 실제 방향과 맞는지 알려 준다.
    같은 줄의 두 방향이 서로 다른 이름이라 따로 남는다 (번호로 다시 그어도 반대 방향은 안 지워짐).
    격자 sideedge 는 줄마다 두 방향이 차례로 나온다. 방향이 없는 예전 이름도 그대로 된다.
"""
import argparse
import datetime
import os
import re
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float64

from record_points import FACES, GRIDS

PRESS_ON = 3.0     # 이 힘[N] 아래는 접촉으로 보지 않고 버린다
MIN_POINTS = 5     # 접촉점이 이보다 적으면 직선 평가 · 방향 확인을 생략한다 (기록은 그대로 남김)

# 면별로 가로·세로 긋기가 고정하는 축과 움직이는 축
#   옆면: 가로 = z 고정·x 로 움직임, 세로 = x 고정·z 로 움직임
#   윗면: 가로 = y 고정·x 로 움직임, 세로 = x 고정·y 로 움직임
LINE_AXES = {
    ('', 'h'): ('옆면', '가로', 'z', 'x'),
    ('', 'v'): ('옆면', '세로', 'x', 'z'),
    ('t', 'h'): ('윗면', '가로', 'y', 'x'),
    ('t', 'v'): ('윗면', '세로', 'x', 'y'),
}

# 센서2 옆면을 바깥에서 볼 때 (아랫면 모서리가 아래) 방향. 10/29 누름 기록으로 확인 (문서 §24)
SENSOR2_SIDE_HINT = {
    '+x': '왼쪽으로', '-x': '오른쪽으로',
    '+z': '위로 (아랫면 모서리에서 멀어짐)', '-z': '아래로 (아랫면 모서리 쪽)',
}

# 격자별로 따로 정한 긋기 줄 (없으면 record_points.py 격자의 줄·칸을 그대로 쓴다)
DRAG_LINES = {
    # 옆면 바깥 띠 (11/03, 사용자 지정). 줄마다 두 방향.
    # 세로 x -100 / 100 / -80 / 80 (8획) + 가로 z -10 ~ 20, 120 ~ 150 (5 간격, 28획) = 36획
    'sideedge': [f'{line}{d}' for line in
                 ['v-100', 'v100', 'v-80', 'v80']
                 + [f'h{z}' for z in (-10, -5, 0, 5, 10, 15, 20,
                                      120, 125, 130, 135, 140, 145, 150)]
                 for d in (('+x', '-x') if line.startswith('h') else ('+z', '-z'))],
}


def parse_label(text):
    """'h40' -> ('옆면', '가로', 'z', 0.040, 'x').  'tv-40' -> ('윗면', '세로', 'x', -0.040, 'y').

    돌려주는 값 = (면, 가로/세로, 고정 축, 자를 댄 자리 [m], 움직이는 축). 못 읽으면 None.
    끝에 방향 (+x, -z 등) 이 붙어 있어도 읽는다. 방향은 parse_direction 으로 따로 꺼낸다.
    """
    m = re.fullmatch(r'(t?)([hv])\s*(-?\d+(?:\.\d+)?)([+-][xyz])?', text.strip(), re.IGNORECASE)
    if not m:
        return None
    face, kind = m.group(1).lower(), m.group(2).lower()
    face_name, kind_name, held, moved = LINE_AXES[(face, kind)]
    if m.group(4) and m.group(4)[1].lower() != moved:
        return None     # 가로줄에 z 방향을 붙이는 식의 잘못된 이름
    return face_name, kind_name, held, float(m.group(3)) / 1000.0, moved


def parse_direction(text):
    """'h150+x' -> +1, 'v-95-z' -> -1, 방향이 없으면 0."""
    m = re.search(r'\d([+-])[xyz]$', text.strip(), re.IGNORECASE)
    if not m:
        return 0
    return 1 if m.group(1) == '+' else -1


def describe(label, sensor=1):
    """'h-5' -> '옆면 가로 z -5 (x 방향으로 긋기)'. 'h150+x' -> '... (x 커지는 쪽으로 긋기)'."""
    target = parse_label(label)
    if target is None:
        return label
    face_name, kind_name, held, value, moved = target
    sign = parse_direction(label)
    if sign == 0:
        how = f'{moved} 방향으로 긋기'
    else:
        how = f'{moved} {"커지는" if sign > 0 else "작아지는"} 쪽으로 긋기'
        hint = SENSOR2_SIDE_HINT.get(f'{"+" if sign > 0 else "-"}{moved}')
        if sensor == 2 and face_name == '옆면' and hint:
            how += f', 바깥에서 보면 {hint}'
    return f'{face_name} {kind_name} {held} {value * 1000.0:g} ({how})'


def build_lines(name):
    """격자 이름 -> [(번호, 줄 이름), ...]. 가로줄 먼저, 그다음 세로줄.

    record_points.py 의 격자 정의를 그대로 쓴다. 면 끝 자리(네 번째 항목)는 줄이 아니라 뺀다.
    """
    if name in DRAG_LINES:
        return [(i + 1, label) for i, label in enumerate(DRAG_LINES[name])]
    face, xs, bs = GRIDS[name][0], GRIDS[name][1], GRIDS[name][2]
    prefix = 't' if face == 't' else ''
    labels = [f'{prefix}h{b}' for b in bs] + [f'{prefix}v{x}' for x in xs]
    return [(i + 1, label) for i, label in enumerate(labels)]


class DragRecorder(Node):
    def __init__(self, sensor, ft_topic):
        super().__init__('record_drag')
        self.rows = []
        self.misses = 0    # 힘은 PRESS_ON 을 넘는데 접촉점이 (0,0,0) 으로 나온 횟수
        self.miss_rows = []  # 그 순간들 (rows 와 같은 열)
        self.active = False
        self.label = ''
        self.mag = float('nan')
        self.ft = [float('nan')] * 6
        self.t0 = 0.0
        self.create_subscription(
            PointStamped, f'/sensor{sensor}/contactsensing/cp', self._cp, 100)
        self.create_subscription(
            Float64, f'/sensor{sensor}/contactsensing/force_magnitude', self._mag, 100)
        self.create_subscription(WrenchStamped, ft_topic, self._ft, 100)

    def _mag(self, m):
        self.mag = m.data

    def _ft(self, m):
        w = m.wrench
        self.ft = [w.force.x, w.force.y, w.force.z,
                   w.torque.x, w.torque.y, w.torque.z]

    def _cp(self, m):
        if not self.active:
            return
        p = (m.point.x, m.point.y, m.point.z)
        if not (self.mag >= PRESS_ON):
            return
        # 접촉이 없을 때 발행되는 (0,0,0) 은 본 기록에서 뺀다. 다만 눌렀는데 안 나온 것은 세고,
        # 원본 렌치는 따로 남긴다 (<이름>_안나옴.csv, 배율 적합에 쓸 수 있게)
        if abs(p[0]) + abs(p[1]) + abs(p[2]) < 1e-9:
            self.misses += 1
            self.miss_rows.append((self.label, time.time() - self.t0, *p, *self.ft))
            return
        self.rows.append((self.label, time.time() - self.t0, *p, *self.ft))

    def start(self, label):
        self.label = label
        self.t0 = time.time()
        self.active = True

    def stop(self):
        self.active = False


def direction_check(rows, moved_axis, sign):
    """줄 이름의 방향과 실제로 그은 방향 (접촉점 기준) 이 맞는지. 맞으면 빈 문자열."""
    if sign == 0 or len(rows) < MIN_POINTS:
        return ''
    k = {'x': 2, 'y': 3, 'z': 4}[moved_axis]
    n = max(1, len(rows) // 5)
    head = sorted(r[k] for r in rows[:n])[n // 2]
    tail = sorted(r[k] for r in rows[-n:])[n // 2]
    if (tail - head) * sign > 0:
        return ''
    return (f'   [주의] 이름은 {moved_axis} {"커지는" if sign > 0 else "작아지는"} 쪽인데 접촉점은 '
            f'{head * 1000:+.0f} -> {tail * 1000:+.0f} mm 로 움직였습니다. 방향이 맞는지 확인하세요.\n')


def summarize(rows, held_axis, target, moved_axis, misses=0):
    """기록된 점들이 직선에서 얼마나 벗어났는지."""
    import statistics
    idx = {'x': 2, 'y': 3, 'z': 4}
    held = [r[idx[held_axis]] for r in rows]          # 일정해야 하는 축
    moved = [r[idx[moved_axis]] for r in rows]
    total = len(held) + misses
    miss_text = (f'   접촉점 안 나옴 {misses}회 (눌린 동안의 {100.0 * misses / total:.0f}%)'
                 if total else '')
    if len(held) < MIN_POINTS:
        return f'   점이 {len(held)}개뿐이라 평가 생략' + ('\n' + miss_text if misses else '')
    err = [(v - target) * 1000.0 for v in held]
    spread = (max(moved) - min(moved)) * 1000.0
    rms = (sum(e * e for e in err) / len(err)) ** 0.5
    return (f'   점 {len(held)}개,  그은 길이 {spread:.0f} mm '
            f'({moved_axis} {min(moved) * 1000:+.0f} ~ {max(moved) * 1000:+.0f})\n'
            f'   자리({held_axis}) 평균 {statistics.mean(held) * 1000:+.1f} mm '
            f'(목표 {target * 1000:+.0f})   흔들림 폭 {max(err) - min(err):.1f} mm\n'
            f'   직선에서 벗어난 양  RMS {rms:.1f} mm   최대 {max(abs(e) for e in err):.1f} mm\n'
            + miss_text)


def print_lines(name, lines, sensor=1):
    face_name = FACES[GRIDS[name][0]][0]
    print(f'   격자 {name} — {face_name} {len(lines)}줄')
    for number, label in lines:
        print(f'     {number:>3}  {describe(label, sensor)}')
    print()


def record_line(node, label):
    """한 줄을 긋는다. 원본 렌치가 남은 순간 수 (접촉점 나옴 + 안 나옴) 를 돌려준다."""
    target = parse_label(label)
    before, misses_before = len(node.rows), node.misses
    node.start(label)
    input('   기록 중... 다 그으면 Enter > ')
    node.stop()
    got = node.rows[before:]
    print(f'   {len(got)} 점 기록')
    print(summarize(got, target[2], target[3], target[4],
                    node.misses - misses_before))
    print(direction_check(got, target[4], parse_direction(label)))
    return len(got) + node.misses - misses_before


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor', type=int, default=1, choices=(1, 2))
    parser.add_argument('--ft-topic', default=None)
    parser.add_argument('--out', default=None,
                        help='기본값: ~/drag_<날짜시각>.csv, 격자면 ~/drag_s<센서>_<격자>_<날짜시각>.csv')
    parser.add_argument('--grid', default=None,
                        choices=sorted(n for n in GRIDS if GRIDS[n][0] in ('s', 't')),
                        help='record_points.py 격자의 줄·칸을 따라 차례로 긋는다')
    args = parser.parse_args()

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    if args.out:
        out_path = args.out
    elif args.grid:
        out_path = os.path.expanduser(f'~/drag_s{args.sensor}_{args.grid}_{stamp}.csv')
    else:
        out_path = os.path.expanduser(f'~/drag_{stamp}.csv')
    ft_topic = args.ft_topic or f'/ur10skku/ftdata_{args.sensor}'

    rclpy.init()
    node = DragRecorder(args.sensor, ft_topic)
    spin = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    print(f'F/T 원본 토픽: {ft_topic}')
    print('자를 대고 직선으로 그으세요. 한 번에 한 줄씩.')
    runs = []
    lines = build_lines(args.grid) if args.grid else []
    done = {}     # 격자 번호 -> 남은 점 수
    nxt = 1       # 격자에서 Enter 로 그을 차례
    if lines:
        print_lines(args.grid, lines, args.sensor)
        print('  Enter 로 시작, 다 그으면 Enter 로 종료.  번호 + Enter = 그 줄 다시.  q + Enter 로 전체 종료.\n')
    else:
        print('  이름 예)  h40   옆면 가로로 긋기, 자를 z=40 mm 에 댐')
        print('            v-50  옆면 세로로 긋기, 자를 x=-50 mm 에 댐')
        print('            th15  윗면 가로로 긋기, 자를 y=15 mm 에 댐')
        print('  이름 + Enter 로 시작, 다 그으면 Enter 로 종료.  q + Enter 로 전체 종료.\n')
    try:
        while True:
            if lines and nxt <= len(lines):
                prompt = (f'[{nxt}/{len(lines)}] {describe(lines[nxt - 1][1], args.sensor)} '
                          f'(Enter=시작, 번호=그 줄, q=종료) > ')
            elif lines:
                prompt = '격자 끝까지 옴. 번호=그 줄 (다시) 긋기, q=종료 > '
            else:
                prompt = '줄 이름 (q=종료) > '
            text = input(prompt).strip()
            if text.lower() == 'q':
                break

            number = None
            if lines and text == '':
                if nxt > len(lines):
                    continue
                number = nxt
            elif lines and text.isdigit():
                number = int(text)
                if not 1 <= number <= len(lines):
                    print(f'   번호는 1 ~ {len(lines)}\n')
                    continue
            if number is not None:
                label = lines[number - 1][1]
                if number in done:
                    # 같은 줄을 다시 그으면 앞 기록을 지운다 (csv 에 같은 이름이 두 번 남지 않게)
                    old = sum(1 for r in node.rows if r[0] == label)
                    node.rows = [r for r in node.rows if r[0] != label]
                    node.miss_rows = [r for r in node.miss_rows if r[0] != label]
                    runs = [r for r in runs if r[0] != label]
                    print(f'   {number}번 앞 기록 {old}점 지움')
                print(f'   {number}번  {describe(label, args.sensor)}')
            else:
                label = text
                if parse_label(label) is None:
                    print('   못 읽었습니다. 예: h40  /  v-50  /  th15  /  tv-40  /  h150+x  /  v-95-z\n')
                    continue

            got = record_line(node, label)
            runs.append((label, got))
            if number is not None:
                # 원본 렌치는 몇 개든 그대로 둔다. 한 순간도 안 눌렸을 때만 안 그은 줄로 남긴다
                if got > 0:
                    done[number] = got
                else:
                    done.pop(number, None)
                    print(f'   눌린 순간이 없어 {number}번은 안 그은 줄로 남깁니다 '
                          f'(다시 그으려면 {number} + Enter).\n')
                # Enter 는 바로 다음 번호로 간다
                if number >= nxt:
                    nxt = number + 1
    except (KeyboardInterrupt, EOFError):
        node.stop()

    with open(out_path, 'w', encoding='utf-8') as h:
        h.write('label,t_sec,cp_x,cp_y,cp_z,Fx,Fy,Fz,Mx,My,Mz\n')
        for r in node.rows:
            h.write(r[0] + ',' + ','.join(f'{v:.6f}' for v in r[1:]) + '\n')
    print(f'\n저장: {out_path}   (총 {len(node.rows)} 점, {len(runs)} 줄)')
    if node.miss_rows:
        miss_path = os.path.splitext(out_path)[0] + '_안나옴.csv'
        with open(miss_path, 'w', encoding='utf-8') as h:
            h.write('label,t_sec,cp_x,cp_y,cp_z,Fx,Fy,Fz,Mx,My,Mz\n')
            for r in node.miss_rows:
                h.write(r[0] + ',' + ','.join(f'{v:.6f}' for v in r[1:]) + '\n')
        print(f'      접촉점 안 나온 순간 {len(node.miss_rows)} 점: {miss_path}')
    if lines:
        missing = [n for n, _ in lines if n not in done]
        if missing:
            print('   안 그은 줄: ' + ', '.join(f'{n}({lines[n - 1][1]})' for n in missing))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
