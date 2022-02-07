import rclpy
from rclpy.node import Node


from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.create_subscription(CompressedImage, '/simulator/camera_node/image/compressed',
                self.camera_callback, 10)


    def camera_callback(self, data):
        image_format = data.format
        image_data = np.array(data.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        print('image format: ', image_format,', size: ',image.shape)
        cv2.imwrite('tmp.jpg', image)
        cv2.imshow('windows', image)
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


