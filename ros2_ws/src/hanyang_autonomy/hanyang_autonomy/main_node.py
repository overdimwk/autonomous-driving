"""Participant starter: sensor input and safe zero-command output; no solution."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32
from orda_interfaces.msg import OrdaDriveCommand

class HanyangAutonomy(Node):
    def __init__(self):
        super().__init__('hanyang_autonomy')
        self.image = self.scan = self.sensor_stamp = None
        self.speed_mps = 0.0
        self.sequence = 0
        self.create_subscription(Image, '/orda/camera/front/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/orda/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Float32, '/orda/vehicle/speed', self.on_speed, qos_profile_sensor_data)
        self.publisher = self.create_publisher(OrdaDriveCommand, '/orda/vehicle/command', 1)
        self.create_timer(0.05, self.tick)  # Recommended command rate: 20 Hz.

    def remember_stamp(self, stamp):
        if self.sensor_stamp is None or (stamp.sec, stamp.nanosec) > (self.sensor_stamp.sec, self.sensor_stamp.nanosec):
            self.sensor_stamp = stamp

    def on_image(self, msg):
        self.image = msg
        self.remember_stamp(msg.header.stamp)

    def on_scan(self, msg):
        self.scan = msg
        self.remember_stamp(msg.header.stamp)

    def on_speed(self, msg):
        self.speed_mps = msg.data

    def control(self):
        # Implement perception, planning and control within this package.
        # Return target speed (m/s), steering command (-100..100).
        return 0.0, 0.0

    def tick(self):
        if self.sensor_stamp is None:
            return
        speed, steering = self.control()
        if not math.isfinite(speed) or not math.isfinite(steering):
            speed = steering = 0.0
        msg = OrdaDriveCommand()
        msg.header.stamp = self.sensor_stamp
        msg.header.frame_id = 'base_link'
        self.sequence += 1
        msg.sequence = self.sequence
        msg.target_speed_mps = float(speed)
        msg.steering_command = float(steering)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HanyangAutonomy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
