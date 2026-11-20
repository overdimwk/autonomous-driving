"""Read-only front camera and planar LiDAR PNG snapshots for development."""
import argparse
import math
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

def main(args=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='/tmp/orda-sensors')
    output=Path(parser.parse_args(args).output)
    output.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=[])
    node=Node('orda_sensor_snapshot')
    captured=set()
    def camera(msg):
        if msg.encoding!='rgb8' or 'camera' in captured: return
        rows=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)
        rgb=rows[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if not cv2.imwrite(str(output/'camera.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)):
            raise RuntimeError('Camera PNG could not be written')
        captured.add('camera')
    def lidar(msg):
        if 'lidar' in captured: return
        canvas=np.full((600,600,3),245,dtype=np.uint8)
        origin=(300,570)
        cv2.circle(canvas,origin,5,(30,30,220),-1)
        for i,distance in enumerate(msg.ranges):
            if not math.isfinite(distance): continue
            angle=msg.angle_min+i*msg.angle_increment
            point=(int(300-distance*math.sin(angle)*14),int(570-distance*math.cos(angle)*14))
            cv2.circle(canvas,point,3,(90,120,30),-1)
        if not cv2.imwrite(str(output/'lidar.png'),canvas):
            raise RuntimeError('LiDAR PNG could not be written')
        captured.add('lidar')
    node.create_subscription(Image,'/orda/camera/front/image_raw',camera,qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/orda/scan',lidar,qos_profile_sensor_data)
    try:
        deadline=time.monotonic()+30
        while len(captured)<2 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        if len(captured)<2: raise RuntimeError('Sensor snapshot timeout')
        print(output)
    finally:
        node.destroy_node()
        rclpy.shutdown()
