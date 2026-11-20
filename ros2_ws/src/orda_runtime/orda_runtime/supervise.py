"""Organizer supervisor; rejects concurrent command publishers and stops the launch."""
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node('orda_execution_monitor')
    child = None
    try:
        # DDS discovery needs a short grace period; do not start a second controller.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.count_publishers('/orda/vehicle/command'):
            raise RuntimeError('An existing vehicle command publisher must be stopped.')
        child = subprocess.Popen(
            ['ros2','launch','hanyang_autonomy','participant.launch.py'], start_new_session=True)
        deadline = time.monotonic() + 30.0
        while child.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.2)
            count = node.count_publishers('/orda/vehicle/command')
            if count > 1:
                raise RuntimeError('Multiple vehicle command publishers are not allowed.')
            if time.monotonic() > deadline and count == 0:
                raise RuntimeError('No vehicle command publisher appeared within 30 seconds.')
        if child.returncode:
            raise RuntimeError(f'Participant launch exited with code {child.returncode}.')
    finally:
        if child is not None and child.poll() is None:
            import os
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
