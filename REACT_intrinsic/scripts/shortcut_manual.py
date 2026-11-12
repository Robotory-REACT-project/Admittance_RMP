#!/usr/bin/env python3
"""단축어 모드 손으로 켜기 시험. 트리거 대신 Enter 로 모드를 켜고, 5초 동안 그린 것을 O / X / 기타로 판정해 찍는다.

    # 1) 센서 (FTGetMain) — 관절값이 들어와야 영점을 잡는다. 로봇 정지 상태로 약 15초 기다림
    ros2 launch Y2FT_AQ ft_get_main.launch.py
    # 2) 접촉점 (main_ifs 두 개)
    ros2 launch REACT_intrinsic react_intrinsic_JSshape.launch.py
    # 3) 이 스크립트 (소스에서 직접 실행)
    python3 ~/ros2_ws/src/REACT_intrinsic/scripts/shortcut_manual.py

    Enter      -> 단축어 모드 켜짐, 5초 동안 커버에 그린다 -> 판정 출력
    q + Enter  -> 끝

판정은 실시간 노드(shortcut_node.py)와 같은 코드다 (docs/shortcut_core.py 의 5초 모드 + gesture_classifier).
로봇 정지 확인과 트리거는 쓰지 않는다. 토픽도 발행하지 않는다.
"""
import argparse
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

SAMPLE_HZ = 100.0          # main_ifs 발행 주기, record_gesture.py 와 같게
DEFAULT_DOCS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', 'docs')
NAMES = {'O': 'O', 'X': 'X', 'N': '기타'}


class SensorState:
    def __init__(self):
        self.cp = (0.0, 0.0, 0.0)
        self.contact = False
        self.mag = float('nan')
        self.cp_count = 0


class ManualShortcut(Node):
    def __init__(self, core, mode):
        super().__init__('shortcut_manual')
        self.core = core
        self.mode = mode
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.events = []
        self.state = {s: SensorState() for s in core.SENSORS}
        for s in core.SENSORS:
            ns = f'/sensor{s}/contactsensing'
            st = self.state[s]
            self.create_subscription(PointStamped, ns + '/cp',
                                     lambda m, st=st: self._cp(st, m), 100)
            self.create_subscription(Bool, ns + '/contact_state',
                                     lambda m, st=st: setattr(st, 'contact', m.data), 100)
            self.create_subscription(Float64, ns + '/force_magnitude',
                                     lambda m, st=st: setattr(st, 'mag', m.data), 100)
        self.create_timer(1.0 / SAMPLE_HZ, self._tick)

    def _cp(self, st, m):
        st.cp = (m.point.x, m.point.y, m.point.z)
        st.cp_count += 1

    def start(self):
        with self.lock:
            self.events = []
            self.done.clear()
            self.mode.activate(time.monotonic(), 'Enter')

    def _tick(self):
        with self.lock:
            if not self.mode.active:
                return
            samples = {s: (st.cp, st.contact, st.mag) for s, st in self.state.items()}
            for ev in self.mode.step(time.monotonic(), samples, still=True):
                self.events.append(ev)
                if ev['type'] == 'mode' and not ev['on']:
                    self.done.set()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--docs-dir', default=DEFAULT_DOCS,
                        help='shortcut_core.py, gesture_classifier.py 가 있는 폴더')
    parser.add_argument('--model', default=None, help='기본: gesture_classifier.DEFAULT_MODEL')
    args, ros_args = parser.parse_known_args()

    sys.path.insert(0, os.path.realpath(args.docs_dir))
    import gesture_classifier as gc
    import shortcut_core as core

    print('모델 불러오는 중...')
    clf = gc.GestureClassifier(args.model or gc.DEFAULT_MODEL)
    mode = core.ShortcutMode(clf, core.Trigger())     # 트리거 안 씀 (Enter 로 켬)

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = ManualShortcut(core, mode)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    time.sleep(2.0)
    for s in core.SENSORS:
        st = node.state[s]
        ok = st.cp_count > 0
        print(f'센서{s} cp: {"들어옴" if ok else "안 들어옴 (main_ifs 확인)"}'
              + (f', 힘 {st.mag:.2f} N' if ok and st.mag == st.mag else ''))
    print(f'\nEnter -> {core.WINDOW_SEC:g}초 동안 커버에 그리기 -> 판정.  q + Enter 로 끝\n')

    n = 0
    try:
        while True:
            if input('[대기] > ').strip().lower() == 'q':
                break
            n += 1
            node.start()
            print(f'   #{n} 단축어 모드 켜짐 — {core.WINDOW_SEC:g}초 동안 그리세요', flush=True)
            if not node.done.wait(core.WINDOW_SEC + 3.0):
                print('   판정이 안 끝났습니다 (센서 토픽 확인)\n')
                continue
            with node.lock:
                events = list(node.events)
            for ev in events:
                if ev['type'] != 'result':
                    continue
                probs = ('O {:.2f}  X {:.2f}'.format(*ev['probs'])
                         if ev['probs'] is not None else '')
                print(f'   => {NAMES[ev["label"]]}   ({probs}  센서{ev["sensor"]}, '
                      f'크기 {ev["size"]:.0f} mm, {ev["why"]})\n')
    except (KeyboardInterrupt, EOFError):
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
