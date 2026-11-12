#!/usr/bin/env python3
"""잔류힘 측정 도구.

누르고 떼는 것을 자동으로 감지해서, 뗀 뒤 남는 잔류힘을 표로 쌓는다.
두드리기·기다리기·반대로 누르기 같은 실험은 아무 때나 글자를 치고 Enter 로 표시한다.

쓰는 법:
    python3 residual_log.py                 # 센서 1
    python3 residual_log.py --sensor 2

    누르고 떼면            -> 자동으로 한 줄 기록 (최대 힘, 누른 시간 포함)
    글자 치고 Enter        -> 그 시점에 표시를 남김 (예: 두드림, 3분대기, 역방향)
    Ctrl+C                 -> 표 출력 + csv 저장

저장 파일:
    ~/residual_<시각>.csv      표 (기록·표시 한 줄씩)
    ~/residual_<시각>_ts.csv   전체 시계열 (SERIES_HZ, elapsed_sec 는 표와 같은 기준)
"""
import argparse
import datetime
import os
import select
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float64

PRESS_ON = 3.0        # 이 값을 넘으면 "누르는 중"으로 본다 [N]
REST_UNDER = 2.0      # 이 아래로 내려오면 뗀 것으로 본다 [N]
SETTLE_SEC = 1.5      # 뗀 뒤 이만큼 기다렸다가
AVERAGE_SEC = 1.0     # 이만큼 평균 내서 잔류로 기록한다
SERIES_HZ = 100.0     # 시계열 저장 주기. 토픽은 2 kHz 라 그대로 두면 1분에 약 10 MB


class Residual(Node):
    def __init__(self, topic, algo_topic):
        super().__init__('residual_log')
        self.buf = []          # (t, fx..mz)
        self.series = []       # SERIES_HZ 로 솎은 전체 기록 (t, fx..mz)
        self.peak = 0.0        # reset_peak() 이후 들어온 |F| 최댓값
        self.algo = float('nan')     # 알고리즘이 내보내는 |F| (비교용)
        self.create_subscription(WrenchStamped, topic, self._cb, 50)
        self.create_subscription(Float64, algo_topic, self._algo, 20)

    def _algo(self, m):
        self.algo = m.data

    def _cb(self, m):
        w = m.wrench
        row = (time.time(),
               w.force.x, w.force.y, w.force.z,
               w.torque.x, w.torque.y, w.torque.z)
        self.buf.append(row)
        if len(self.buf) > 20000:
            del self.buf[:10000]
        if not self.series or row[0] - self.series[-1][0] >= 1.0 / SERIES_HZ:
            self.series.append(row)
        # 메인 루프는 몇 ms 마다 최신값만 보므로, 최댓값은 모든 샘플에서 센다
        self.peak = max(self.peak, (row[1] ** 2 + row[2] ** 2 + row[3] ** 2) ** 0.5)

    def reset_peak(self):
        self.peak = self.mag()

    def mag(self):
        if not self.buf:
            return 0.0
        _, fx, fy, fz = self.buf[-1][:4]
        return (fx * fx + fy * fy + fz * fz) ** 0.5

    def window(self, seconds):
        """최근 seconds 초의 6축 중앙값과 힘 크기의 표준편차."""
        now = time.time()
        rows = [r for r in self.buf if now - r[0] <= seconds]
        if len(rows) < 5:
            return None, None
        med = [statistics.median(r[i] for r in rows) for i in range(1, 7)]
        mags = [(r[1] ** 2 + r[2] ** 2 + r[3] ** 2) ** 0.5 for r in rows]
        return med, statistics.pstdev(mags)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor', type=int, default=1, choices=(1, 2))
    parser.add_argument('--topic', default=None)
    args = parser.parse_args()
    topic = args.topic or f'/ur10skku/ftdata_{args.sensor}'
    algo_topic = f'/sensor{args.sensor}/contactsensing/force_magnitude'

    rclpy.init()
    node = Residual(topic, algo_topic)
    rows = []
    start = time.time()
    press_count = 0
    state = 'rest'
    release_at = 0.0
    press_at = 0.0
    watch_stdin = sys.stdin.isatty()      # 파이프로 돌리면 표시 기능은 끈다

    print(f'토픽: {topic}')
    print(f'누르고 떼면 자동 기록. 글자+Enter 로 표시. Ctrl+C 로 종료.\n')
    print(f'{"시각":>7} {"구분":>10} {"|F|원본":>8}{"|F|알고":>8}  '
          f'{"Fx":>7}{"Fy":>7}{"Fz":>7}{"Mx":>8}{"My":>8}{"Mz":>8}'
          f'{"최대|F|":>8}{"누른s":>6}')

    def emit(kind, peak=float('nan'), press_sec=float('nan')):
        med, sd = node.window(AVERAGE_SEC)
        if med is None:
            return
        mag = (med[0] ** 2 + med[1] ** 2 + med[2] ** 2) ** 0.5
        el = time.time() - start
        rows.append((el, kind, mag, node.algo, *med, peak, press_sec))
        print(f'{el:7.1f} {kind:>10} {mag:8.3f}{node.algo:8.3f}  '
              f'{med[0]:7.3f}{med[1]:7.3f}{med[2]:7.3f}'
              f'{med[3]:8.4f}{med[4]:8.4f}{med[5]:8.4f}'
              f'{peak:8.2f}{press_sec:6.2f}')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            # 키보드 표시
            if watch_stdin and select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line == '':            # EOF
                    watch_stdin = False
                else:
                    emit((line.strip() or '표시')[:10])

            m = node.mag()
            if state == 'rest' and m > PRESS_ON:
                state = 'press'
                press_at = time.time()
                node.reset_peak()
            elif state == 'press' and m < REST_UNDER:
                state = 'settle'
                release_at = time.time()
            elif state == 'settle':
                if m > PRESS_ON:
                    state = 'press'
                elif time.time() - release_at > SETTLE_SEC:
                    press_count += 1
                    emit(f'{press_count}회후', node.peak, release_at - press_at)
                    state = 'rest'
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        # Humble의 rclpy가 Ctrl+C 종료와 메시지 수신이 겹칠 때
        # take_message()에서 RuntimeError를 낼 수 있다. 이미 모은 행은 보존한다.
        print(f'\nROS 메시지 수신 중 종료됨: {error}', file=sys.stderr)
    finally:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        if rows:
            out = os.path.expanduser(f'~/residual_{stamp}.csv')
            with open(out, 'w', encoding='utf-8') as f:
                f.write('elapsed_sec,label,mag_raw_N,mag_algo_N,Fx,Fy,Fz,Mx,My,Mz,'
                        'peak_N,press_sec\n')
                for r in rows:
                    f.write(
                        ','.join(
                            f'{v:.5f}' if isinstance(v, float) else str(v) for v in r) + '\n')
            print(f'\n{len(rows)}줄 저장: {out}')
        else:
            print('\n저장할 측정 행이 없습니다.')
        if node.series:
            out_ts = os.path.expanduser(f'~/residual_{stamp}_ts.csv')
            with open(out_ts, 'w', encoding='utf-8') as f:
                f.write('elapsed_sec,Fx,Fy,Fz,Mx,My,Mz\n')
                for r in node.series:
                    f.write(f'{r[0] - start:.4f},' + ','.join(f'{v:.5f}' for v in r[1:]) + '\n')
            print(f'시계열 {len(node.series)}줄 저장: {out_ts}')

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
