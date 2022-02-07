import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point
from lgsvl_msgs.msg import VehicleControlData
from sensor_msgs.msg import CompressedImage
import math
import cv2
import numpy as np
from matplotlib import pyplot as plt

DIS_THRESHOLD = 5.0  # TODO: tune this to stop at right last point
DIS_BACK_THRESHOLD = 3.0  # stop avoiding
DIS_TURN_THRESHOLD = 20.0  # start avoiding
RAD_THRESHOLD = 0.1


class CheckPoint:
    '''To calculate checkpoint turn rad'''
    def __init__(self, waypoint_x, waypoint_z):
        self.waypoint_x = waypoint_x
        self.waypoint_z = waypoint_z
        self.dis = None
        self.previous_x = None
        self.previous_z = None
        self.turn_rad = None

    def _signed_rad_btw_vectors(self, v_goal, v_current):
        unsigned_rad = math.acos((v_goal[0] * v_current[0] + v_goal[1] * v_current[1]) / (math.hypot(*v_goal) * math.hypot(*v_current) + 1e-8))
        if (v_goal[0] * v_current[1] - v_goal[1] * v_current[0]) < 0:
            signed_rad = -unsigned_rad
        else:
            signed_rad = unsigned_rad
        return signed_rad

    def update(self, current_x, current_z):
        dis = math.hypot(current_x - self.waypoint_x, current_z - self.waypoint_z)
        
        turn_rad = 0.  # set turn_rad default to 0 (straitgh forward)
        if self.previous_x is not None and self.previous_z is not None:
            v_goal = [self.waypoint_x - current_x, self.waypoint_z - current_z]
            v_current = [current_x - self.previous_x, current_z - self.previous_z]
            turn_rad = self._signed_rad_btw_vectors(v_goal, v_current)
        
        if self.dis is None or dis < self.dis:
            self.dis = dis
            self.turn_rad = turn_rad
        
        self.previous_x = current_x
        self.previous_z = current_z

    def get_dis(self):
        return self.dis

    def get_turn_rad(self):
        return self.turn_rad


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
        self.pub = self.create_publisher(VehicleControlData,
                                         '/lgsvl/vehicle_control_cmd', 10)
        self.create_subscription(CompressedImage, '/lgsvl/camera',
                                 self.camera_callback, 10)

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

        # load waypoints
        self.check_points = []
        # with open("waypoints.csv") as fp:
        #     for line in fp:
        #         waypoint_x, waypoint_z = [float(v) for v in line.split(",")]
        #         self.check_points.append(CheckPoint(waypoint_x, waypoint_z))
        self.check_points.append(CheckPoint(86.95, 35.66))  # Start avoiding point
        self.check_points.append(CheckPoint(53.64, 61.24))  # End point
        self.num_points = len(self.check_points)
        assert self.num_points > 0
        
        self.tmr = self.create_timer(0.11, self.controller_callback)

        self.img = None
        self.left_curvem, self.center_offset = None, None
        self.brake_period = 1

        # store current position_callback
        self.current_x = None
        self.current_z = None

        self.point_index = 0

    def controller_callback(self):
        msg = VehicleControlData()
        msg.acceleration_pct = 0.12
        msg.braking_pct = 0.0

        if self.img is not None:
            self.lane_detect(self.img)
            if self.center_offset is not None:
                if self.center_offset + 10000 > 13000:
                    msg.target_wheel_angle = -0.2
                    # print("turn left")
                    msg.braking_pct = 0.0
                elif self.center_offset + 10000 < -2000:
                    msg.target_wheel_angle = 0.2
                    # print("turn right")
                    msg.braking_pct = 0.0
                else:
                    self.brake_period += 1
                    # print(self.brake_period)
                    if self.brake_period % 100 == 0 or (self.brake_period > 250 and self.brake_period % 25 == 0):
                        msg.braking_pct = 0.4
                        print("Break in straight line")
                    msg.target_wheel_angle = 0.
        
        if self.point_index < self.num_points - 1 and self.current_x is not None and self.current_x is not None:
            curr_point = self.check_points[self.point_index]
            
            # update distance to current and previous check point
            curr_point.update(self.current_x, self.current_z)
            
            # msg.target_wheel_angle = curr_point.get_turn_rad()
            # if abs(msg.target_wheel_angle) > RAD_THRESHOLD:
            #     msg.braking_pct = 0.15
            
            # print(msg.target_wheel_angle, self.point_index, curr_point.get_dis())
            # print(f"{curr_point.get_dis()=}")
            if curr_point.get_dis() < DIS_TURN_THRESHOLD:
                print("Start avoiding")
                msg.target_wheel_angle = curr_point.get_turn_rad()
            if curr_point.get_dis() < DIS_BACK_THRESHOLD:
                self.point_index += 1
        
        elif self.point_index == self.num_points - 1:  # last point, stop
            # print(f"{self.point_index=}")
            curr_point = self.check_points[self.point_index]
            curr_point.update(self.current_x, self.current_z)
            if curr_point.get_dis() < DIS_THRESHOLD:
                msg.acceleration_pct = 0.0
                msg.braking_pct = 1.0
                    
        self.pub.publish(msg)

    def position_callback(self, data):
        # TODO: store vehicle position
        self.current_x = data.x
        self.current_z = data.z

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
