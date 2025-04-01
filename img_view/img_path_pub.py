#!/usr/bin/env python
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String

import numpy as np
import cv2
from cv_bridge import CvBridge
import json

# 이미지 메시지 데이터를 어레이 형태로 변환
bridge = CvBridge() 

class ImageSubscriber(Node) :
   def __init__(self) :
     super().__init__('image_path_')
     self.image_sub = self.create_subscription(
        Image, # 임포트 된 메시지 타입 
        '/image_raw', # 토픽리스트에서 조회한 토픽 주소
        self.image_callback, # 정의한 콜백함수
        10)
     self.publisher_ = self.create_publisher(String, '/aimd/imgs', 10)
     self.image = np.empty(shape=[1])


   def image_callback(self, data) :
     self.image = bridge.imgmsg_to_cv2(data, 'bgr8')
     t = time.time_ns()
     filename = f"{t//1_000_000_000}.{t%1_000_000_000:09d}" + ".jpg"
     file_path = os.path.join(directory, filename)
     cv2.imwrite(file_path, self.image)
#     cv2.imshow('img', self.image)
#     cv2.waitKey(33)
     msg = String()
     dict = {'file_path':file_path}
     msg.data = json.dumps(dict)
     self.publisher_.publish(msg)
     
def main(args=None) :
  rclpy.init(args=args)
  node = ImageSubscriber()

  try :
    print("Going to spin.")
    rclpy.spin(node)
  except KeyboardInterrupt :
    node.get_logger().info('Stopped by Keyboard')
  finally :
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__' :
  main()
