from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from lgsvl_msgs.msg import VehicleControlData
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import math
import time

DIS_THRESHOLD = 0.15  
DIS_END_THRESHOLD = 0.15 # TODO: tune this to stop at right last point
DIS_START_BREAK_THRESHOLD = 7.0
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


class Driver(Node):
    def __init__(self):
        super().__init__("Driver")
        self.pub = self.create_publisher(
            VehicleControlData, "/lgsvl/vehicle_control_cmd", 10
        )
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

        # self.create_subscription(CompressedImage, '/lgsvl/camera',
            # self.camera_callback, 10)

        # load waypoints
        sparse_waypoints = [
            [2.42, 22.34],
            [2.41 ,22.34],
            [2.40 ,22.33],
            [2.28 ,22.29],
            [2.15 ,22.24],
            [2.02 ,22.20],
            [1.89 ,22.15],
            [1.76 ,22.11],
            [1.64 ,22.06],
            [1.51 ,22.02],
            [1.38 ,22.01],
            [1.26 ,22.02],
            [1.16 ,22.01],
            [1.05 ,21.99],
            [0.95 ,21.96],
            [0.84 ,21.95],
            [0.74 ,21.93],
            [0.64 ,21.91],
            [0.54 ,21.91],
            [0.44 ,21.91],
            [0.35 ,21.92],
            [0.13 ,21.92],
            [-0.02 ,21.90],
            [-0.18 ,21.87],
            [-0.35 ,21.86],
            [-0.50 ,21.84],
            [-0.66 ,21.84],
            [-0.82 ,21.83],
            [-1.06 ,21.80],
            [-1.36 ,21.76],
            [-1.66 ,21.72],
            [-1.96 ,21.70],
            [-2.26 ,21.66],
            [-2.55 ,21.64],
            [-2.84 ,21.61],
            [-3.14 ,21.58],
            [-3.28 ,21.56],
            [-3.57 ,21.53],
            [-3.86 ,21.50],
            [-4.15 ,21.47],
            [-4.44 ,21.44],
            [-4.73 ,21.41],
            [-5.01 ,21.37],
            [-5.30 ,21.34],
            [-5.58 ,21.31],
            [-5.87 ,21.28],
            [-6.15 ,21.25],
            [-6.43 ,21.22],
            [-6.71 ,21.19],
            [-6.99 ,21.16],
            [-7.27 ,21.13],
            [-7.55 ,21.09],
            [-7.82 ,21.05],
            [-8.08 ,20.98],
            [-8.35 ,20.94],
            [-8.61 ,20.89],
            [-8.85 ,20.80],
            [-9.09 ,20.71],
            [-9.34 ,20.66],
            [-9.59 ,20.60],
            [-9.950 ,20.48],
            [-10.18 ,20.41],
            [-10.41 ,20.34],
            [-10.65 ,20.28],
            [-10.88 ,20.22],
            [-11.12 ,20.16],
            [-11.35 ,20.09],
            [-11.58 ,20.03],
            [-11.81 ,19.97],
            [-12.04 ,19.90],
            [-12.26 ,19.82],
            [-12.48 ,19.75],
            [-12.81 ,19.65],
            [-13.03 ,19.59],
            [-13.25 ,19.52],
            [-13.45 ,19.46],
            [-13.47 ,19.46],
            [-13.47 ,19.46],
            [-13.47 ,19.46],
            [-13.47 ,19.46],
            [-13.47 ,19.46],
            [-13.47 ,19.46],
            [-13.47 ,19.46]]
        self.check_points = []
        # with open("sparse_waypoints.csv") as fp:
        #     for i, line in enumerate(fp):
        #         if i % 2 == 0:
        #             waypoint_x, waypoint_z = [float(v) for v in line.split(",")]
        #             self.check_points.append(CheckPoint(waypoint_x, waypoint_z))
        for waypoint_x, waypoint_z in sparse_waypoints:
            self.check_points.append(CheckPoint(waypoint_x, waypoint_z))
        # self.check_points.append(CheckPoint(-64.88, -6.84))
        # self.check_points.append(CheckPoint(-54.55, -9.35))
        # self.check_points.append(CheckPoint(-42.16 ,-4.9))  # End point
        self.num_points = len(self.check_points)
        assert self.num_points > 0
        
        self.tmr = self.create_timer(0.15, self.controller_callback)
        
        # store current position_callback
        self.current_x = None
        self.current_z = None
        self.current_t = None
        
        self.point_index = 0

        self.brake_npc = False

        self.prev_x = 0.
        self.prev_z = 0.
        self.prev_t = 0.

        self.control_index = 0

    def controller_callback(self):
        # TODO: implement your car controller

        msg = VehicleControlData()
        if self.point_index == 0:
            msg.acceleration_pct = 0.2
        else:
            msg.acceleration_pct = 0.1
        msg.braking_pct = 0.0
        print(f"{self.point_index=}")
        if self.point_index < self.num_points and self.current_x is not None and self.current_x is not None:
            curr_point = self.check_points[self.point_index]
            end_point = self.check_points[-1]
            
            # update distance to current and previous check point
            curr_point.update(self.current_x, self.current_z)
            end_point.update(self.current_x, self.current_z)
            
            msg.target_wheel_angle = curr_point.get_turn_rad()
            if abs(msg.target_wheel_angle) and self.point_index != 0 > RAD_THRESHOLD:
                msg.braking_pct = 0.1
            
            # print(msg.target_wheel_angle, self.point_index, curr_point.get_dis())
            
            print(f"{curr_point.get_dis()=}")
            if curr_point.get_dis() < DIS_THRESHOLD:
                self.point_index += 1
            
            if end_point.get_dis() < DIS_END_THRESHOLD:
                msg.acceleration_pct = 0.0
                msg.braking_pct = 1.0
            
            print(f"{self.control_index=}")
            # if self.point_index == 0 and self.control_index > 190 and self.control_index < 270:
            if self.point_index == 0 and curr_point.get_dis() < DIS_START_BREAK_THRESHOLD and self.control_index < 270:
                msg.braking_pct = 1.0
            self.control_index += 1


        elif self.point_index == self.num_points:  # last point, stop
            msg.acceleration_pct = 0.0
            msg.braking_pct = 1.0

        if self.brake_npc:
            msg.acceleration_pct = 0.0
            msg.braking_pct = 1.0
        
        self.pub.publish(msg)

    def position_callback(self, data):
        # TODO: store vehicle position
        self.current_x = data.x
        self.current_z = data.z
        self.current_t = time.time()
        # print("Current pos: %f, %f, %f" % (data.x, data.y, data.z))

    def camera_callback(self, data):
        image_format = data.format
        image_data = np.array(data.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        # print('image format: ', image_format,', size: ',image.shape)
        # self.sparse_waypoints.append([self.current_x, self.current_z])
        # cv2.imwrite(f'tmp_{self.index}.jpg', image)
        # cv2.imshow('windows', image)
        img_hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        npc_mask = cv2.inRange(img_hls, (0/2, 0.52*255, 0.01*255), (359/2, 0.59*255, 0.06*255))
        cnts = cv2.findContours(npc_mask[500:, 500:], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        self.brake_npc = False
        for c in cnts:
            if cv2.contourArea(c) > 2000:
                print(f"Brake: {cv2.contourArea(c)=}")
                # self.brake_npc = True
        # cv2.imshow('windows', cv2.resize(mask_yello, (960, 540)))
        # cv2.waitKey(2000)
        # self.index += 1


def main():
    rclpy.init()
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
