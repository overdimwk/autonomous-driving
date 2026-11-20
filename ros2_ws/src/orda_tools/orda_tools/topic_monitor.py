"""Read-only sensor stream monitor; no steering or hidden state."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32

def main(args=None):
    rclpy.init(args=args)
    node=Node('orda_topic_monitor')
    counts={'camera':0,'lidar':0,'speed':0}
    def callback(name):
        def receive(msg):
            counts[name]+=1
        return receive
    for cls, topic, name in [(Image,'/orda/camera/front/image_raw','camera'),
                              (LaserScan,'/orda/scan','lidar'),
                              (Float32,'/orda/vehicle/speed','speed')]:
        node.create_subscription(cls,topic,callback(name),qos_profile_sensor_data)
    started=time.monotonic()
    def report():
        nonlocal started
        elapsed=time.monotonic()-started
        node.get_logger().info(' '.join(f'{k}={v/elapsed:.1f}Hz' for k,v in counts.items()))
        for k in counts:
            counts[k]=0
        started=time.monotonic()
    node.create_timer(2.0,report)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
