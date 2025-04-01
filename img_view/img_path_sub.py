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
        String, # 임포트 된 메시지 타입 
        '/aimd/imgs', # 토픽리스트에서 조회한 토픽 주소
        self.path_callback, # 정의한 콜백함수
        10)
     self.image = np.empty(shape=[1])


   def path_callback(self, msg) :
     print("msg received : ", msg.data)
     message = msg.data
     dict = json.loads(message)
     file_path = dict['file_path']
     image = cv2.imread(file_path)
     cv2.imshow("image_from_path", image)
     cv2.waitKey(33)
     
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
