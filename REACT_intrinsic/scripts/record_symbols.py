#!/usr/bin/env python3
"""숫자 (와 O · X · 기타) 기록기. 화면에 나온 글자를 커버에 손가락으로 쓰면 그 궤적과 F/T 를 남긴다 (10/27).

record_gesture.py 와 같은 노드 · 같은 csv 형식이다 (그 파일을 import 해서 쓴다. record_gesture.py 는 안 바꿈).
다른 점:
    - 쓸 글자를 이 스크립트가 정해 보여 준다. 한 바퀴에 모든 글자가 한 번씩 나오고 바퀴마다 순서를 섞는다
      (중간에 그만둬도 글자마다 개수가 고르다)
    - 라벨에 숫자 1 ~ 9 가 있다 (0 은 O 와 같아 안 씀). O · X · 기타(N) 도 --symbols 로 섞을 수 있다
    - 판정 결과를 보여 주지 않는다 (쓰는 사람이 모델에 맞춰 글씨를 바꾸지 않게).
      판정은 나중에 docs/gesture_classifier.py 에 이 csv 를 넣어서 본다
    - 기타 (N) 차례에는 OTHER_KINDS 의 모양을 차례로 보여 준다 (10/29). n 번째 기타 = OTHER_KINDS[(n - 1) % len(OTHER_KINDS)].
      csv 에는 라벨 N 만 남으므로 노트북은 파일 안 기타의 순서로 모양을 안다 (d 로 지우면 같은 모양을 다시 묻는다)
    - --free-other (10/29 센서 PC 추가): 기타를 OTHER_KINDS 순서와 상관없이 자유롭게 쓴다. 화면에는 '자유 모양' 만
      보여 주고, csv 옆에 <csv 이름>_기타자유.txt 를 남겨 이 파일의 기타 순서로 모양을 알 수 없다고 적는다.
      csv 형식은 그대로다

쓰는 법 (이 폴더에서):
    python3 record_symbols.py --face side --writer p1                  # 1 ~ 9 를 10번씩 (90개)
    python3 record_symbols.py --face side --writer p1 --per 5          # 5번씩
    python3 record_symbols.py --face side --writer p1 --symbols OX123456789N
    python3 record_symbols.py --face side --writer kth --symbols N --per 40    # 기타만 40개 (모양 20가지 × 2)
    python3 record_symbols.py --face side --writer kth --symbols N --per 40 --free-other   # 기타 40개, 모양 자유

    Enter          -> 보이는 글자를 쓴다 -> 손 떼고 Enter
    d + Enter      -> 방금 것 지우기 (그 글자를 바로 다시 묻는다)
    s + Enter      -> 이 글자 건너뛰기 (맨 뒤로 보냄)
    q + Enter      -> 끝 (그때까지 쓴 것은 저장돼 있다)

    한 글자마다 csv 를 통째로 다시 쓴다. 저장: ~/gesture_symbols_<writer>_<면>_<시각>.csv
"""
import argparse
import datetime
import os
import random
import threading
import time

import rclpy

import record_gesture as rg

SYMBOLS_OK = 'OXN123456789'
MIN_SIZE_MM = 20.0         # gesture_classifier.MIN_SIZE_MM 과 같게 (10/29: 5 → 20). 이보다 작으면 판정에서 기타로 빠진다
SHOW = {'N': '기타'}
# 10/29: 기타로 쓸 모양 (순서대로 돌아감). 기호와 헷갈리는 모양은 뺐다
# (세로선 = 1, ㄱ · 갈고리 = 7, 동그라미 · 나선 = O, 교차하는 두 선 = X). 20 mm 보다 작으면 규칙이 거르므로 크게
OTHER_KINDS = (
    '가로선 길게 (8 cm 넘게)',
    '\\ 사선 (왼쪽 위 -> 오른쪽 아래, 약 45도)',
    '/ 사선 (왼쪽 아래 -> 오른쪽 위, 약 45도)',
    '화살표 (-> 또는 <-)',
    'U (위가 열린 반원)',
    '∩ (아래가 열린 반원)',
    '⊂ (오른쪽이 열린 반원)',
    '⊃ (왼쪽이 열린 반원)',
    '가로 물결 ~ (두세 번 출렁)',
    'V',
    '∧ (뒤집은 V)',
    '< 또는 > (꺾쇠)',
    '체크 (✓)',
    '가로 지그재그 (네 번 넘게 꺾기)',
    '= (가로선 두 줄)',
    '여러 자리 톡톡 (2 ~ 4번, 서로 3 cm 넘게 떨어지게)',
    '엉킨 낙서 (고리 · 교차 여러 번)',
    '문지르기 (한 자리를 여러 번 비비기, 약 3 cm)',
    '손바닥으로 쓸기 (손바닥 전체로 밀며 옆으로)',
    '세모 또는 네모 (각진 닫힌 도형)',
)


def make_queue(symbols, per, rng):
    """한 바퀴 = 모든 글자 한 번씩 (섞어서). 그걸 per 바퀴."""
    queue = []
    for _ in range(per):
        one = list(symbols)
        rng.shuffle(one)
        queue += one
    return queue


def counts_text(trials, symbols, per):
    n = {s: 0 for s in symbols}
    for _, label, _ in trials:
        n[label] = n.get(label, 0) + 1
    return '  '.join(f'{s} {n[s]}/{per}' for s in symbols)


def report(rows, label, face):
    """record_gesture.py 와 같은 요약. 판정은 하지 않는다."""
    print(f'   {len(rows)} 샘플 ({len(rows) / rg.SAMPLE_HZ:.1f} s)')
    touched = False
    for s in rg.SENSORS:
        n, strokes, peak, got_face, w, h = rg.summarize(rows, s)
        if n == 0:
            print(f'   센서{s}: 접촉 없음')
            continue
        touched = True
        notes = []
        if got_face != face:
            notes.append(f'면이 {got_face} 로 잡힘')
        expected = rg.EXPECTED_STROKES.get(label)
        if expected is not None and strokes != expected:
            notes.append(f'{label} 는 보통 {expected}획')
        if max(w, h) < MIN_SIZE_MM:
            notes.append(f'{MIN_SIZE_MM:g} mm 보다 작음 (판정에서 기타' + (', d 로 지우고 크게 다시' if label == 'N' else '') + ')')
        note = ('   <- ' + ', '.join(notes)) if notes else ''
        print(f'   센서{s}: 접촉 {n / rg.SAMPLE_HZ:.2f} s, {strokes}획, '
              f'최대 {peak:.1f} N, 크기 {w:.0f} x {h:.0f} mm{note}')
    if not touched:
        print('   접촉이 안 잡혔습니다. d 로 지우고 다시 쓰세요.')
    print()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--face', required=True, choices=('top', 'side'),
                        help='이번에 쓸 면. 기록된 cp 가 다른 면이면 경고한다')
    parser.add_argument('--writer', required=True,
                        help='쓴 사람 (예: p1). 사람마다 따로 평가하려고 꼭 적는다')
    parser.add_argument('--symbols', default='123456789',
                        help=f'쓸 글자 ({SYMBOLS_OK} 중에서, N = 기타). 기본 1 ~ 9')
    parser.add_argument('--per', type=int, default=10, help='글자마다 몇 번 (기본 10)')
    parser.add_argument('--seed', type=int, default=None, help='순서 난수 (기본: 지금 시각)')
    parser.add_argument('--out', default=None)
    parser.add_argument('--free-other', action='store_true',
                        help='기타를 OTHER_KINDS 순서와 상관없이 자유롭게 쓴다. csv 옆에 _기타자유.txt 를 남긴다')
    args = parser.parse_args()

    symbols = args.symbols.upper()
    if any(c not in SYMBOLS_OK for c in symbols) or len(set(symbols)) != len(symbols) or not symbols:
        raise SystemExit(f'--symbols 는 {SYMBOLS_OK} 중에서 겹치지 않게: {args.symbols}')
    if args.per < 1:
        raise SystemExit('--per 는 1 이상')
    writer = args.writer.replace(',', '_')
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = args.out or os.path.expanduser(f'~/gesture_symbols_{writer}_{args.face}_{stamp}.csv')
    seed = args.seed if args.seed is not None else int(time.time())
    queue = make_queue(symbols, args.per, random.Random(seed))
    # --free-other: 기타 모양이 화면 순서와 다르다는 것을 csv 와 같이 가는 파일에 남긴다
    note_path = os.path.splitext(out_path)[0] + '_기타자유.txt'
    note_text = (
        f'{os.path.basename(out_path)} 의 기타 (라벨 N) 는 --free-other 로 기록했다.\n'
        '쓴 사람이 모양을 자유롭게 골랐다. 화면에 OTHER_KINDS 모양을 보여 주지 않았으므로\n'
        'n 번째 기타 = OTHER_KINDS[(n - 1) % 20] 이 아니다. 이 파일의 기타는 순서로 모양을 알 수 없다.\n'
        '쓰지 않기로 한 모양 (세로선 · 세로 물결 = 1, ㄱ · 갈고리 = 7, 동그라미 · 나선 = O, 교차하는 두 선 = X,\n'
        '숫자를 닮은 글자, 한 번 누르기) 은 빼고 썼다.\n')

    rclpy.init()
    node = rg.GestureRecorder()
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    time.sleep(2.0)
    for s in rg.SENSORS:
        ok = node.state[s].cp_count > 0
        print(f'센서{s} cp: {"들어옴" if ok else "안 들어옴 (main_ifs 확인)"}')
    face_name = '윗면' if args.face == 'top' else '옆면'
    print(f'\n{face_name}에 쓰기.  쓴 사람 {writer},  글자 {symbols} × {args.per} = {len(queue)}개 (순서 난수 {seed})')
    print(f'저장: {out_path}')
    print('  Enter 로 시작 -> 보이는 글자를 쓰고 손 뗀 뒤 Enter')
    print('  d + Enter 방금 것 지우기 (다시 묻습니다),  s + Enter 건너뛰기,  q + Enter 끝\n')

    trials = []
    next_id = 1
    try:
        while True:
            if not queue:
                cmd = input(f'[{counts_text(trials, symbols, args.per)}]  다 썼습니다. '
                            'q 끝 / d 마지막 것 지우기 > ').strip().lower()
                if cmd == 'd' and trials:
                    trial_id, label, _ = trials.pop()
                    rg.save(out_path, trials, args.face, writer)
                    queue.insert(0, label)
                    print(f'   #{trial_id} {label} 지움 -> 다시 씁니다\n')
                elif cmd == 'q':
                    break
                continue
            sym = queue[0]
            shown = SHOW.get(sym, sym)
            if sym == 'N':
                k = sum(1 for _, label, _ in trials if label == 'N')      # 이미 남긴 기타 수
                kind = '자유 모양' if args.free_other else OTHER_KINDS[k % len(OTHER_KINDS)]
                shown = f'{shown} {k + 1}: {kind}'
            cmd = input(f'[{counts_text(trials, symbols, args.per)}]  남은 {len(queue)}\n'
                        f'   #{next_id}  쓸 글자:  [ {shown} ]   (Enter 시작) > ').strip().lower()
            if cmd == 'q':
                break
            if cmd == 'd':
                if trials:
                    trial_id, label, _ = trials.pop()
                    rg.save(out_path, trials, args.face, writer)
                    queue.insert(0, label)
                    print(f'   #{trial_id} {label} 지움 -> 다시 씁니다\n')
                else:
                    print('   지울 것이 없습니다\n')
                continue
            if cmd == 's':
                queue.append(queue.pop(0))
                print('   맨 뒤로 보냈습니다\n')
                continue
            if cmd != '':
                print('   Enter / d / s / q 중 하나\n')
                continue

            node.start()
            input(f'   #{next_id} [ {shown} ] 기록 중... 다 쓰고 손 뗀 뒤 Enter > ')
            rows = node.stop()
            trials.append((next_id, sym, rows))
            rg.save(out_path, trials, args.face, writer)
            if args.free_other and sym == 'N' and not os.path.exists(note_path):
                with open(note_path, 'w', encoding='utf-8') as h:
                    h.write(note_text)
            queue.pop(0)
            report(rows, sym, args.face)
            next_id += 1
    except (KeyboardInterrupt, EOFError):
        node.stop()

    if trials:
        print(f'\n저장: {out_path}   ({counts_text(trials, symbols, args.per)})')
        if os.path.exists(note_path):
            print(f'      {note_path}   (기타 모양 자유 표시, csv 와 같이 보낼 것)')
    elif os.path.exists(out_path):
        print(f'\n전부 지웠습니다. 빈 파일: {out_path}')
    else:
        print('\n기록한 글자가 없어 저장하지 않았습니다.')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
