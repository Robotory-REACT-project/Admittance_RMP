#!/usr/bin/env python3
"""F/T 센서 없이 접촉을 흉내 내는 시험용 스크립트.

실제 센서가 없는 노트북에서 좌표계 방향(wrench_calibration_*)을 맞추기 위한 도구다.

동작:
  1. JSshape_surface.stl 에서 넓은 면 몇 곳을 눌러볼 자리로 고른다.
  2. URDF 의 joint origin 을 읽어 "그 서포터에 실제로 붙은 센서라면 어떤 값을
     뱉을지" 를 계산한다. 즉 base_link 기준 접촉을 센서 자기 좌표계로 되돌려서
     발행한다. 그래서 이건 보정값까지 포함한 전체 경로 시험이 된다.
  3. main_ifs 가 되돌린 접촉점을 구독해서 원래 누른 자리와 비교한다.

보정값이 맞으면 오차가 1 mm 근처(코드가 일부러 더하는 값), 틀리면 크게 벌어진다.

쓰는 법:
    # 터미널 1
    ros2 launch REACT_intrinsic react_intrinsic_JSshape.launch.py use_ft_sensors:=false
    # 터미널 2
    python3 fake_ft.py                 # 센서 1,2 를 여러 지점에서 시험하고 채점
    python3 fake_ft.py --hold          # 계속 발행 (RViz 로 화살표를 눈으로 볼 때)
    python3 fake_ft.py --sensor 2      # 한쪽만
"""

import argparse
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, WrenchStamped
from rclpy.node import Node

SHARE = Path(get_package_share_directory('REACT_intrinsic'))
SURFACE_STL = SHARE / 'meshes' / 'JSshape_surface.stl'
URDF = SHARE / 'urdf' / 'react_intrinsic_JSshape.urdf'

# 센서 번호 -> (URDF 링크, 발행할 힘 토픽, 결과를 받을 접촉점 토픽)
SENSORS = {
    1: ('ur10_jsshape_supporter1', '/ur10skku/ftdata_1', '/sensor1/contactsensing/cp'),
    2: ('ur10_jsshape_supporter2', '/ur10skku/ftdata_2', '/sensor2/contactsensing/cp'),
}

FORCE_N = 10.0        # 누르는 힘 [N]. contact_force_threshold(1.0) 보다 커야 한다.
PUBLISH_HZ = 50.0
TOLERANCE_M = 0.004   # 4 mm. 코드가 접촉점 z 에 1 mm 를 일부러 더한다.


def rpy_to_matrix(roll, pitch, yaw):
    """URDF 의 rpy(고정축 R-P-Y) 를 회전행렬로. R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def read_urdf_placement(link_name):
    """base_link 기준 해당 링크의 실제 자세 (R, t) 를 URDF 에서 읽는다."""
    root = ET.parse(URDF).getroot()
    for joint in root.findall('joint'):
        child = joint.find('child')
        if child is None or child.get('link') != link_name:
            continue
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        return rpy_to_matrix(*rpy), np.array(xyz)
    raise SystemExit(f'URDF 에서 {link_name} 을 찾지 못했습니다: {URDF}')


def read_surface_stl():
    """바이너리 STL 에서 (꼭짓점, 법선) 목록을 뽑는다."""
    data = SURFACE_STL.read_bytes()
    count = struct.unpack('<I', data[80:84])[0]
    verts, normals = [], []
    for i in range(count):
        base = 84 + 50 * i
        normals.append(np.array(struct.unpack('<3f', data[base:base + 12])))
        verts.append([np.array(struct.unpack('<3f', data[base + 12 + 12 * k:base + 24 + 12 * k]))
                      for k in range(3)])
    return np.array(verts), np.array(normals)


def ray_hits(verts, normals, origin, direction):
    """작용선이 뚫는 앞면 삼각형 수. Moller-Trumbore."""
    hits = 0
    for i in range(len(verts)):
        unit = normals[i] / np.linalg.norm(normals[i])
        if unit @ direction >= 0:
            continue
        v0, v1, v2 = verts[i]
        edge1, edge2 = v1 - v0, v2 - v0
        h = np.cross(direction, edge2)
        det = edge1 @ h
        if abs(det) < 1e-12:
            continue
        inv = 1.0 / det
        s = origin - v0
        u = inv * (s @ h)
        if u < 0.0 or u > 1.0:
            continue
        q = np.cross(s, edge1)
        v = inv * (direction @ q)
        if v < 0.0 or u + v > 1.0:
            continue
        hits += 1
    return hits


def pick_targets(verts, normals, wanted=3):
    """서로 다른 면에서 넓은 삼각형을 골라 누를 자리로 삼는다.

    한 지점만 보면 판정이 흔들린다. 알고리즘이 이웃 삼각형을 훑다가
    엉뚱한 면에 안착하는 자리가 실제로 있다. 그래서 여러 면을 본다.
    작용선이 표면을 두 번 이상 뚫는 자리는 애초에 답이 하나가 아니라 제외한다.
    """
    centers = verts.mean(axis=1)
    areas = np.array([0.5 * np.linalg.norm(np.cross(v[1] - v[0], v[2] - v[0])) for v in verts])
    picked, seen_faces = [], []
    for idx in np.argsort(-areas):
        unit = normals[idx] / np.linalg.norm(normals[idx])
        if ray_hits(verts, normals, centers[idx] - unit * 1e-4, -unit) != 1:
            continue
        face = tuple(np.round(unit, 2))
        if face in seen_faces:
            continue
        seen_faces.append(face)
        picked.append((centers[idx], unit))
        if len(picked) == wanted:
            break
    if not picked:
        raise SystemExit('누를 만한 표면을 못 찾았습니다. STL 을 확인하세요.')
    return picked


class FakeFt(Node):
    def __init__(self, sensor_ids):
        super().__init__('fake_ft')
        self.placement = {}
        self.pub = {}
        self.sub_value = {}
        self.msg = {}
        self.sent = {}
        for sid in sensor_ids:
            link, ft_topic, cp_topic = SENSORS[sid]
            self.placement[sid] = read_urdf_placement(link)
            self.pub[sid] = self.create_publisher(WrenchStamped, ft_topic, 10)
            self.sub_value[sid] = None
            self.create_subscription(
                PointStamped, cp_topic,
                lambda m, s=sid: self.sub_value.__setitem__(
                    s, np.array([m.point.x, m.point.y, m.point.z])),
                10)
            self.get_logger().info(f'센서{sid} [{link}]  {ft_topic}  ->  {cp_topic}')
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)

    def aim(self, point_local, normal_local):
        """누를 자리를 정하고, 각 센서가 뱉을 값을 계산해 둔다."""
        expected = {}
        for sid, (rot, trans) in self.placement.items():
            # 서포터가 놓인 자세를 반영해 base_link 기준 접촉으로 바꾼다.
            # 힘은 표면을 파고드는 쪽(-법선)이어야 한다. get_ClosestSurface 가
            # 법선과 힘의 내적이 0 이상인 삼각형을 걸러내기 때문이다.
            point = rot @ point_local + trans
            force = rot @ (-normal_local) * FORCE_N
            # 실제 센서가 뱉을 값: 자기 좌표계로 되돌린 힘과 모멘트.
            force_s = rot.T @ force
            torque_s = np.cross(rot.T @ (point - trans), force_s)
            msg = WrenchStamped()
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force_s
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = torque_s
            self.msg[sid] = msg
            expected[sid] = point
            self.sent[sid] = (force_s, torque_s)
        return expected

    def _tick(self):
        now = self.get_clock().now().to_msg()
        for sid, msg in self.msg.items():
            msg.header.stamp = now
            self.pub[sid].publish(msg)

    def spin_for(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_node(self, timeout=15.0):
        """main_ifs 가 우리 토픽을 실제로 구독할 때까지 기다린다.

        연결 전에 채점하면 아직 아무것도 계산 안 된 초기값을 읽고
        멀쩡한 설정도 '어긋남' 으로 나온다.
        """
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(p.get_subscription_count() > 0 for p in self.pub.values()):
                return True
        return False

    def measure(self, seconds):
        """저역통과 필터(lambda=0.05)가 수렴할 시간을 준 뒤 결과를 받는다."""
        self.spin_for(1.5)
        for sid in self.sub_value:
            self.sub_value[sid] = None
        self.spin_for(seconds)
        return dict(self.sub_value)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor', type=int, choices=[1, 2], action='append',
                        help='시험할 센서 (기본: 1과 2 모두)')
    parser.add_argument('--hold', action='store_true',
                        help='채점 없이 계속 발행한다 (RViz 로 눈으로 볼 때)')
    parser.add_argument('--points', type=int, default=3, help='눌러볼 지점 수 (기본 3)')
    parser.add_argument('--seconds', type=float, default=2.5,
                        help='지점당 측정 시간 [s] (기본 2.5)')
    args = parser.parse_args()

    verts, normals = read_surface_stl()
    targets = pick_targets(verts, normals, args.points)

    rclpy.init()
    node = FakeFt(args.sensor or [1, 2])
    try:
        if not node.wait_for_node():
            print('\nmain_ifs 노드가 안 보입니다. 다른 터미널에서 launch 를 먼저 띄우세요:')
            print('  ros2 launch REACT_intrinsic react_intrinsic_JSshape.launch.py '
                  'use_ft_sensors:=false')
            raise SystemExit(1)

        if args.hold:
            node.aim(*targets[0])
            node.get_logger().info('계속 발행 중입니다. Ctrl+C 로 멈추세요.')
            rclpy.spin(node)
            return

        results = {sid: [] for sid in node.pub}
        for number, (point_local, normal_local) in enumerate(targets, start=1):
            expected = node.aim(point_local, normal_local)
            got = node.measure(args.seconds)
            print(f'\n[지점 {number}/{len(targets)}]  표면 법선 {np.round(normal_local, 2)}')
            for sid in sorted(expected):
                exp, actual = expected[sid], got[sid]
                if actual is None:
                    print(f'  센서{sid}: 접촉점이 안 왔습니다.')
                    results[sid].append(None)
                    continue
                err = float(np.linalg.norm(actual - exp))
                mark = 'OK ' if err < TOLERANCE_M else '틀림'
                print(f'  센서{sid}: 보낸힘 {np.round(node.sent[sid][0], 2)}'
                      f'  누른 {np.round(exp, 4)} -> 되찾은 {np.round(actual, 4)}'
                      f'   오차 {err * 1000:6.1f} mm  {mark}')
                results[sid].append(err)

        print('\n' + '=' * 60)
        print('종합')
        print('=' * 60)
        all_ok = True
        for sid in sorted(results):
            errs = [e for e in results[sid] if e is not None]
            passed = sum(1 for e in errs if e < TOLERANCE_M)
            verdict = '정상' if passed == len(targets) else '어긋남'
            if passed != len(targets):
                all_ok = False
            print(f'  센서{sid}: {passed}/{len(targets)} 지점 일치  ->  {verdict}')
        print('=' * 60)
        if not all_ok:
            print('한 지점만 틀리면 그 자리가 애매한 것일 수 있습니다.')
            print('여러 지점이 틀리면 wrench_calibration_* 이 실제 장착과 다릅니다.')
        raise SystemExit(0 if all_ok else 2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
