import rclpy
from rclpy.node import Node

from lgsvl_msgs.msg import VehicleControlData
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class Driver(Node):
    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(VehicleControlData,
                                         '/lgsvl/vehicle_control_cmd', 10)
        self.create_subscription(CompressedImage, '/lgsvl/camera',
                self.camera_callback, 10)

        self.tmr = self.create_timer(1.0, self.controller_callback)

        self.brake_period = 0

    def controller_callback(self):
        # TODO: implement your car controller

        msg = VehicleControlData()
        msg.acceleration_pct = 0.2
        msg.braking_pct = 0.0
        self.brake_period += 1
        print(self.brake_period)
        if self.brake_period % 200 == 0:
            msg.braking_pct = 0.5
            print("Break in straight line")
        msg.target_wheel_angle = 0.

        self.pub.publish(msg)

    def camera_callback(self, data):
        image_format = data.format
        image_data = np.array(data.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        # print('image format: ', image_format,', size: ',image.shape)
        cv2.waitKey(1)
        # TODO: implement your car controller


def main():
    rclpy.init()
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
