#!/usr/bin/env python3
"""
전방 카메라 이미지 수집 스크립트 (YOLO 학습 데이터용)

- 구독: /orda/camera/front/image_raw  (sensor_msgs/Image, 640x360, rgb8)
- 저장: <out>/<세션시각>/<세션시각>_000001.jpg ...  + frames.csv (파일명 ↔ 센서 stamp)
- 저장 간격은 '센서 stamp(Unity 시각)' 기준으로 조절 → 시뮬레이터가 느려져도 간격이 일정
- 차량이 멈춰 있을 때 같은 장면이 쌓이지 않도록 '거의 같은 프레임'은 건너뜀

hanyang_autonomy 패키지 '밖'에 두고 단독으로 실행합니다. (제출 ZIP에 섞이지 않게, 빌드 불필요)

사용 예 (ROS2 환경이 준비된 터미널 = Mac은 Docker 컨테이너 안):
    source /opt/ros/jazzy/setup.bash
    python3 record_camera.py                      # 기본: 초당 2장, ./camera_dataset 에 저장
    python3 record_camera.py --hz 5 --out ./camera_dataset
    python3 record_camera.py --hz 0 --min-diff 0  # 들어오는 프레임 전부 저장
종료: Ctrl+C
"""
import argparse
import csv
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image → OpenCV BGR 배열 (cv_bridge 없이 NumPy로 변환)."""
    h, w, enc = msg.height, msg.width, msg.encoding.lower()
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)  # step: 한 줄의 바이트 수

    if enc in ('rgb8', 'bgr8'):
        img = rows[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else np.ascontiguousarray(img)
    if enc in ('rgba8', 'bgra8'):
        img = rows[:, :w * 4].reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR)
    if enc == 'mono8':
        return cv2.cvtColor(np.ascontiguousarray(rows[:, :w]), cv2.COLOR_GRAY2BGR)
    raise ValueError(f'지원하지 않는 encoding: {msg.encoding}')


class CameraRecorder(Node):
    def __init__(self, args):
        super().__init__('camera_recorder')
        self.args = args

        # 저장 폴더: <out>/<세션시각>/   (out 최상위에 COLCON_IGNORE를 두어 colcon이 이미지 폴더를 뒤지지 않게 함)
        root = os.path.abspath(os.path.expanduser(args.out))
        self.session = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.out_dir = os.path.join(root, self.session)
        os.makedirs(self.out_dir, exist_ok=True)
        open(os.path.join(root, 'COLCON_IGNORE'), 'a').close()

        self.csv_file = open(os.path.join(self.out_dir, 'frames.csv'), 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['file', 'stamp_sec', 'stamp_nanosec', 'width', 'height', 'encoding'])

        self.min_dt = 1.0 / args.hz if args.hz > 0 else 0.0
        self.last_save_t = None      # 마지막으로 저장한 프레임의 센서 시각 [s]
        self.last_small = None       # 중복 판정용 축소 흑백 이미지
        self.saved = self.received = self.skipped_dup = 0
        self.window_count = 0
        self.window_start = time.monotonic()
        self.done = False

        if args.format == 'jpg':
            self.ext, self.write_params = '.jpg', [cv2.IMWRITE_JPEG_QUALITY, args.quality]
        else:
            self.ext, self.write_params = '.png', [cv2.IMWRITE_PNG_COMPRESSION, 3]

        # 센서 토픽은 best effort로 구독 (퍼블리셔가 reliable이든 best effort든 수신 가능)
        self.create_subscription(Image, args.topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f'구독: {args.topic} | 저장 폴더: {self.out_dir} | '
            f'간격: {"전부" if self.min_dt == 0 else f"{args.hz}장/초"} | 중복 임계값: {args.min_diff}')

    def on_image(self, msg: Image):
        self.received += 1
        self.window_count += 1
        if self.done:
            return

        # 1) 센서 시각 기준 저장 간격 조절 (시뮬레이터 재시작으로 시각이 뒤로 가면 리셋)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_save_t is not None and t >= self.last_save_t and t - self.last_save_t < self.min_dt:
            return

        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        # 2) 거의 같은 장면이면 건너뜀 (정지 상태에서 같은 사진이 쌓이는 것 방지)
        small = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (64, 36), interpolation=cv2.INTER_AREA)
        if self.args.min_diff > 0 and self.last_small is not None:
            diff = float(np.mean(cv2.absdiff(small, self.last_small)))
            if diff < self.args.min_diff:
                self.skipped_dup += 1
                return

        # 3) 저장
        name = f'{self.session}_{self.saved + 1:06d}{self.ext}'
        if not cv2.imwrite(os.path.join(self.out_dir, name), bgr, self.write_params):
            self.get_logger().error(f'저장 실패: {name}', throttle_duration_sec=5.0)
            return
        self.csv.writerow([name, msg.header.stamp.sec, msg.header.stamp.nanosec,
                           msg.width, msg.height, msg.encoding])
        self.saved += 1
        self.last_save_t = t
        self.last_small = small

        if self.args.max > 0 and self.saved >= self.args.max:
            self.done = True
            self.csv_file.flush()
            self.get_logger().info(f'최대 {self.args.max}장 저장 완료. Ctrl+C로 종료하세요.')

    def report(self):
        now = time.monotonic()
        rate = self.window_count / (now - self.window_start)
        self.window_count, self.window_start = 0, now
        self.csv_file.flush()
        if self.received == 0:
            self.get_logger().warn(
                '카메라 메시지가 아직 없습니다. 시뮬레이터에 ROS2 LINK ACTIVE가 떠 있는지 확인하세요 '
                '(수동연습.command는 ROS 연결 없이 실행되므로 토픽이 나오지 않습니다).')
            return
        self.get_logger().info(
            f'수신 {rate:5.1f} Hz | 저장 {self.saved}장 | 중복 건너뜀 {self.skipped_dup}장')

    def close(self):
        if not self.csv_file.closed:
            self.csv_file.close()


def main():
    p = argparse.ArgumentParser(description='전방 카메라 이미지 수집 (YOLO 학습용)')
    p.add_argument('--topic', default='/orda/camera/front/image_raw')
    p.add_argument('--out', default='./camera_dataset', help='저장 최상위 폴더 (세션별 하위 폴더 생성)')
    p.add_argument('--hz', type=float, default=2.0, help='초당 저장 장수 (센서 시각 기준). 0이면 전부 저장')
    p.add_argument('--min-diff', type=float, default=2.0,
                   help='직전 저장 이미지와의 평균 밝기 차이(0~255)가 이보다 작으면 건너뜀. 0이면 끔')
    p.add_argument('--max', type=int, default=0, help='최대 저장 장수. 0이면 제한 없음')
    p.add_argument('--format', choices=['jpg', 'png'], default='jpg')
    p.add_argument('--quality', type=int, default=95, help='JPG 품질 (1~100)')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = CameraRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'종료: {node.saved}장 저장 → {node.out_dir}')
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
