from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from lgsvl_msgs.msg import VehicleControlData
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import pandas as pd
import math

class Driver(Node):
    def __init__(self):
        super().__init__("Driver")
        self.create_subscription(
            Point,
            "/ground_truth/vehicle_position",
            self.position_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1,
            ),
        )

        self.create_subscription(CompressedImage, '/lgsvl/camera',
            self.camera_callback, 10)
        self.index = 0
        self.sparse_waypoints = []
                
        # store current position_callback
        self.current_x = None
        self.current_z = None


    def position_callback(self, data):
        # TODO: store vehicle position
        self.current_x = data.x
        self.current_z = data.z
        print("Current pos: %f, %f, %f" % (data.x, data.y, data.z))

    def camera_callback(self, data):
        image_format = data.format
        image_data = np.array(data.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        print('image format: ', image_format,', size: ',image.shape)
        self.sparse_waypoints.append([self.current_x, self.current_z])
        cv2.imwrite(f'tmp_{self.index}.jpg', image)
        # cv2.imshow('windows', image)
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_yello = cv2.inRange(img_hsv, (14, 127, 0), (20, 255, 255))
        cv2.imshow('windows', cv2.resize(mask_yello, (960, 540)))
        cv2.waitKey(2000)
        self.index += 1
        #TODO


def main():
    rclpy.init()
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        df = pd.DataFrame(node.sparse_waypoints) # construct data frame and transpose
        df.to_csv('sparse_waypoints.csv', header=False, index=False, sep=',')
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
