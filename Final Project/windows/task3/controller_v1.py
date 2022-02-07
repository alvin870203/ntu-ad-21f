import rclpy
from rclpy.node import Node

from lgsvl_msgs.msg import VehicleControlData
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from matplotlib import pyplot as plt


class Lane:
    def __init__(self, orig_frame):
        self.orig_frame = orig_frame
        self.lane_line_markings = None
        self.warped_frame = None
        self.transformation_matrix = None
        self.inv_transformation_matrix = None
        self.orig_image_size = self.orig_frame.shape[::-1][1:]
        width = self.orig_image_size[0]
        self.roi_points = np.float32([
            (841,  577),
            (100,  901),
            (1407, 901),
            (1053, 577)
        ])
        self.padding = int(0.25 * width)
        self.desired_roi_points = np.float32([
            [self.padding,                         0],
            [self.padding,                         self.orig_image_size[1]],
            [self.orig_image_size[0]-self.padding, self.orig_image_size[1]],
            [self.orig_image_size[0]-self.padding, 0]
        ])
        self.histogram = None
        self.XM_PER_PIX = 1920 / 1920
        self.center_offset = None
        
        self.seen_left_x = 50

    def calculate_car_position(self, print_to_terminal=False):
        car_location = self.orig_frame.shape[1] / 2
        bottom_left = self.histogram_peak()
        if bottom_left is None:
            return False  # nothing detected
        center_lane = bottom_left + 240
        center_offset = (np.abs(car_location) - np.abs(center_lane)) * self.XM_PER_PIX * 100
        self.center_offset = center_offset
        return center_offset

    def calculate_histogram(self, frame=None):
        if frame is None:
            frame = self.warped_frame
        # print(len(frame), len(frame[0]))
        self.histogram = np.sum(frame[int(frame.shape[0]/2):, :], axis=0)  # tune this to decide area to see
        # print(len(self.histogram))

    def my_get_line_markings(self, frame=None):
        if frame is None:
            frame = self.orig_frame
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_yello = cv2.inRange(img_hsv, (14, 127, 0), (20, 255, 255))
        self.lane_line_markings = mask_yello

    def histogram_peak(self):
        # roi_bottom_left_x = int(self.desired_roi_points[1][0])
        roi_bottom_left_x = self.seen_left_x
        if np.amax(self.histogram[roi_bottom_left_x:]) > 3000:  # yellow detected
            leftx_base = np.argmax(self.histogram[roi_bottom_left_x:]) + roi_bottom_left_x
        else:  # nothing detected
            leftx_base = None
        # print(roi_bottom_left_x)
        # leftx_base = np.argmax(self.histogram[roi_bottom_left_x:]) + roi_bottom_left_x
        print(f"{np.amax(self.histogram[roi_bottom_left_x:])}")
        return leftx_base

    def perspective_transform(self, frame=None):
        if frame is None:
            frame = self.lane_line_markings
        self.transformation_matrix = cv2.getPerspectiveTransform(self.roi_points, self.desired_roi_points)
        self.inv_transformation_matrix = cv2.getPerspectiveTransform(self.desired_roi_points, self.roi_points)
        self.warped_frame = cv2.warpPerspective(frame, self.transformation_matrix, self.orig_image_size, flags=(cv2.INTER_LINEAR))
        (thresh, binary_warped) = cv2.threshold(self.warped_frame, 127, 255, cv2.THRESH_BINARY)
        self.warped_frame = binary_warped


class Driver(Node):
    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(VehicleControlData,
                                         '/lgsvl/vehicle_control_cmd', 10)
        self.create_subscription(CompressedImage, '/lgsvl/camera',
                                 self.camera_callback, 10)

        self.tmr = self.create_timer(0.11, self.controller_callback)

        self.img = None
        self.left_curvem, self.center_offset = None, None
        self.brake_period = 1

        self.left_bias = 14000
        self.left_dist = 43000

    def controller_callback(self):

        msg = VehicleControlData()
        msg.acceleration_pct = 0.15
        msg.braking_pct = 0.0

        if self.img is not None:
            self.lane_detect(self.img)
            if self.center_offset is not None:
                if self.center_offset is False:
                    print("No yellow line detected, go straight")
                    msg.target_wheel_angle = 0.
                elif self.center_offset + self.left_bias > self.left_dist:
                    msg.target_wheel_angle = -0.25
                    print(f"turn left; {self.center_offset + self.left_bias =}")
                    msg.braking_pct = 0.0
                elif self.center_offset + self.left_bias < -2000:
                    msg.target_wheel_angle = 0.2
                    print("turn right")
                    msg.braking_pct = 0.0
                else:
                    self.brake_period += 1
                    # print(self.brake_period)
                    print("straight")
                    if self.brake_period % 200 == 0 or (self.brake_period > 350 and self.brake_period < 400):
                        msg.braking_pct = 0.15
                        print("Break in straight line")
                    msg.target_wheel_angle = 0.
                    
        self.pub.publish(msg)

    def camera_callback(self, data):
        image_format = data.format
        image_data = np.array(data.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        cv2.waitKey(1)
        self.img = image

    def lane_detect(self, img):
        lane_obj = Lane(orig_frame=img)
        lane_obj.my_get_line_markings()
        lane_obj.perspective_transform()
        lane_obj.calculate_histogram()
        self.center_offset = lane_obj.calculate_car_position()


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
