#!/usr/bin/env python3
"""
[컨테이너 안에서 실행됨] 카메라 토픽 → JPEG 프레임 스트림 (YOLO 검출 검증용)

ORDA 참가자 컨테이너는 네트워크가 막혀 있고 /submission은 읽기 전용이라서,
프레임을 '표준 출력(stdout)'으로 흘려보내고 Mac의 yolo_viewer.py가 `docker exec`로 받아 갑니다.
보통은 직접 실행할 필요 없이 yolo_viewer.py가 자동으로 이 스크립트를 실행합니다.

    [Unity] → /orda/camera/front/image_raw → frame_bridge.py --stdout ──(docker exec 파이프)──▶ Mac: yolo_viewer.py

- 추가 설치 필요 없음 (rclpy, numpy, OpenCV는 컨테이너에 이미 있음)
- 받는 쪽이 느리면 오래된 프레임은 버리고 '가장 최신 프레임'만 보냄 → 지연이 쌓이지 않음
- stdout에는 프레임 데이터만 나가고, 상태 메시지는 전부 stderr로 출력 (Mac 뷰어 터미널에 보임)
- 받는 쪽(뷰어)이 꺼지면 자동 종료

모드:
    --stdout            (기본 사용) 표준 출력으로 전송 — docker exec 파이프용
    --host H --port P   TCP로 전송 — 컨테이너 네트워크가 열려 있는 환경용
"""
import argparse
import os
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# 프레임 하나 = [헤더 24바이트: 'ORDA' | seq(uint64) | 센서 stamp(float64) | JPEG 길이(uint32)] + JPEG 바이트
HEADER = struct.Struct('!4sQdI')
MAGIC = b'ORDA'


def log(text):
    """상태 메시지는 항상 stderr로 (stdout은 프레임 데이터 전용)."""
    print(text, file=sys.stderr, flush=True)


def image_to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image → OpenCV BGR (cv_bridge 없이)."""
    h, w, enc = msg.height, msg.width, msg.encoding.lower()
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)
    if enc in ('rgb8', 'bgr8'):
        img = rows[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else np.ascontiguousarray(img)
    if enc in ('rgba8', 'bgra8'):
        img = rows[:, :w * 4].reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR)
    if enc == 'mono8':
        return cv2.cvtColor(np.ascontiguousarray(rows[:, :w]), cv2.COLOR_GRAY2BGR)
    raise ValueError(f'지원하지 않는 encoding: {msg.encoding}')


class LatestSlotSender(threading.Thread):
    """최신 프레임 1장만 보관하다가 보내는 스레드의 공통 뼈대. send()만 하위 클래스에서 구현."""

    def __init__(self):
        super().__init__(daemon=True)
        self.cond = threading.Condition()
        self.item = None
        self.running = True
        self.connected = False
        self.sent = 0

    def put(self, item):
        with self.cond:
            self.item = item              # 아직 못 보낸 이전 프레임은 덮어씀(버림)
            self.cond.notify()

    def stop(self):
        with self.cond:
            self.running = False
            self.cond.notify()

    def run(self):
        while self.running:
            with self.cond:
                if self.item is None:
                    self.cond.wait(timeout=0.5)
                item, self.item = self.item, None
            if item is not None and self.send(item):
                self.sent += 1

    @staticmethod
    def pack(item):
        seq, stamp, jpeg = item
        return HEADER.pack(MAGIC, seq, stamp, len(jpeg)) + jpeg


class StdoutSender(LatestSlotSender):
    """표준 출력으로 전송 (docker exec 파이프). 받는 쪽이 끊기면 프로세스 종료."""

    def __init__(self):
        super().__init__()
        self.out = sys.stdout.buffer
        self.connected = True

    def send(self, item):
        try:
            self.out.write(self.pack(item))
            self.out.flush()
            return True
        except (BrokenPipeError, OSError):
            log('[bridge] 받는 쪽(뷰어)이 종료되어 브리지도 종료합니다.')
            os._exit(0)


class TcpSender(LatestSlotSender):
    """TCP로 전송. 연결이 끊기면 자동 재연결."""

    def __init__(self, host, port):
        super().__init__()
        self.host, self.port = host, port
        self.sock = None
        self._last_err = 0.0

    def send(self, item):
        if self.sock is None:
            try:
                self.sock = socket.create_connection((self.host, self.port), timeout=2.0)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.connected = True
                log(f'[bridge] 뷰어에 연결됨: {self.host}:{self.port}')
            except OSError as e:
                self.connected = False
                if time.monotonic() - self._last_err > 5.0:
                    log(f'[bridge] 뷰어 연결 대기 중... ({self.host}:{self.port}, {e})')
                    self._last_err = time.monotonic()
                time.sleep(1.0)
                return False
        try:
            self.sock.sendall(self.pack(item))
            return True
        except OSError as e:
            log(f'[bridge] 전송 실패, 재연결합니다: {e}')
            self.sock.close()
            self.sock, self.connected = None, False
            return False


class FrameBridge(Node):
    def __init__(self, args):
        super().__init__('frame_bridge')
        self.args = args
        self.sender = StdoutSender() if args.stdout else TcpSender(args.host, args.port)
        self.sender.start()
        self.min_dt = 1.0 / args.max_fps if args.max_fps > 0 else 0.0
        self.last_t = 0.0
        self.seq = 0
        self.received = 0
        self.window_start = time.monotonic()
        self.create_subscription(Image, args.topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)
        target = 'stdout(docker exec 파이프)' if args.stdout else f'{args.host}:{args.port}'
        log(f'[bridge] 구독: {args.topic} → {target} (최대 {args.max_fps} fps, JPEG 품질 {args.quality})')

    def on_image(self, msg: Image):
        self.received += 1
        now = time.monotonic()
        if now - self.last_t < self.min_dt:
            return
        self.last_t = now
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            log(f'[bridge] {e}')
            return
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self.args.quality])
        if not ok:
            return
        self.seq += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.sender.put((self.seq, stamp, buf.tobytes()))

    def report(self):
        now = time.monotonic()
        rate = self.received / (now - self.window_start)
        self.received, self.window_start = 0, now
        if rate == 0:
            log('[bridge] 카메라 메시지가 없습니다. 시뮬레이터에 ROS2 LINK ACTIVE가 떠 있는지 확인하세요.')
        else:
            state = '연결됨' if self.sender.connected else '연결 대기'
            log(f'[bridge] 카메라 수신 {rate:5.1f} Hz | 전송 누적 {self.sender.sent}장 | 뷰어 {state}')


def main():
    p = argparse.ArgumentParser(description='카메라 토픽 → YOLO 뷰어로 프레임 전송')
    p.add_argument('--topic', default='/orda/camera/front/image_raw')
    p.add_argument('--stdout', action='store_true', help='표준 출력으로 전송 (docker exec 파이프용)')
    p.add_argument('--host', default='host.docker.internal')
    p.add_argument('--port', type=int, default=5005)
    p.add_argument('--max-fps', type=float, default=30.0, help='초당 최대 전송 장수 (0이면 제한 없음)')
    p.add_argument('--quality', type=int, default=90, help='JPEG 품질 1~100')
    args, ros_args = p.parse_known_args()

    if args.stdout and sys.stdout.isatty():
        log('[bridge] --stdout 모드는 터미널에 직접 실행하는 용도가 아닙니다. Mac에서 yolo_viewer.py --docker 로 실행하세요.')
        sys.exit(1)

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = FrameBridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        log(f'[bridge] 종료: 총 {node.sender.sent}장 전송')
        node.sender.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
