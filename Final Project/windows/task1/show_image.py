import rclpy
from rclpy.node import Node


from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.create_subscription(CompressedImage, '/lgsvl/camera',
                self.camera_callback, 10)


    def camera_callback(self, data):
        image_format = data.format
        image_data = np.array(data.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        print('image format: ', image_format,', size: ',image.shape)
        cv2.imwrite('tmp.jpg', image)
        # cv2.imshow('windows', image)
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_yello = cv2.inRange(img_hsv, (14, 127, 0), (20, 255, 255))
        cv2.imshow('windows', cv2.resize(mask_yello, (960, 540)))
        cv2.waitKey(1)
        #TODO

rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


