import rclpy
from rclpy.node import Node


from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


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

    def calculate_car_position(self, print_to_terminal=False):
        car_location = self.orig_frame.shape[1] / 2
        bottom_left = self.histogram_peak()
        center_lane = bottom_left + 240
        center_offset = (np.abs(car_location) - np.abs(center_lane)) * self.XM_PER_PIX * 100
        self.center_offset = center_offset
        return center_offset

    def calculate_histogram(self, frame=None):
        if frame is None:
            frame = self.warped_frame
        self.histogram = np.sum(frame[int(frame.shape[0]/2):, :], axis=0)

    def my_get_line_markings(self, frame=None):
        if frame is None:
            frame = self.orig_frame
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_yello = cv2.inRange(img_hsv, (14, 127, 0), (20, 255, 255))
        self.lane_line_markings = mask_yello

    def histogram_peak(self):
        roi_bottom_left_x = int(self.desired_roi_points[1][0])
        leftx_base = np.argmax(self.histogram[roi_bottom_left_x:]) + roi_bottom_left_x
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
        self.create_subscription(CompressedImage, '/lgsvl/camera',
                self.camera_callback, 10)


    def camera_callback(self, data):
        image_format = data.format
        image_data = np.array(data.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        print('image format: ', image_format,', size: ',image.shape)
        cv2.imwrite('tmp.jpg', image)
        # cv2.imshow('windows', image)

        lane_obj = Lane(orig_frame=image)
        lane_obj.my_get_line_markings()
        lane_obj.perspective_transform()
        cv2.imshow('windows', lane_obj.warped_frame[int(lane_obj.warped_frame.shape[0]/2):, 50:])


        # img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # mask_yello = cv2.inRange(img_hsv, (14, 127, 0), (20, 255, 255))
        # cv2.imshow('windows', cv2.resize(mask_yello, (960, 540)))
        
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


