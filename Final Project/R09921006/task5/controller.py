from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from lgsvl_msgs.msg import VehicleControlData
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import math

DIS_THRESHOLD = 1.0  
DIS_END_THRESHOLD = 2.0 # TODO: tune this to stop at right last point
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

        self.create_subscription(CompressedImage, '/lgsvl/camera',
            self.camera_callback, 10)

        # load waypoints
        self.check_points = []
        self.check_points = [
            [36.94,24.57],
            [40.78,25.11],
            [44.06,24.23],
            [47.64,22.47],
            [51.47,21.75],
            [55.50,22.31],
            [59.03,24.44],
            [61.11,27.56],
            [62.40,30.94],
            [62.51,35.11],
            [61.61,38.14],
            [59.84,40.90],
            [57.39,42.80],
            [54.91,43.81],
            [52.18,44.29],
            [48.56,43.99],
            [44.98,42.46],
            [42.25,40.02],
            [38.91,36.26],
            [35.50,35.00]
        ]
        self.check_points = [CheckPoint(*p) for p in self.check_points]
        # with open("sparse_waypoints.csv") as fp:
        #     for line in fp:
        #         waypoint_x, waypoint_z = [float(v) for v in line.split(",")]
        #         self.check_points.append(CheckPoint(waypoint_x, waypoint_z))
        # for waypoint_x, waypoint_z in sparse_waypoints:
        #     self.check_points.append(CheckPoint(waypoint_x, waypoint_z))
        # self.check_points.append(CheckPoint(-64.88, -6.84))
        # self.check_points.append(CheckPoint(-54.55, -9.35))
        # self.check_points.append(CheckPoint(-42.16 ,-4.9))  # End point
        self.num_points = len(self.check_points)
        assert self.num_points > 0
        
        self.tmr = self.create_timer(0.15, self.controller_callback)
        
        # store current position_callback
        self.current_x = None
        self.current_z = None
        
        self.point_index = 0

        self.brake_npc = False

        self.index = 0

    def controller_callback(self):
        # TODO: implement your car controller

        msg = VehicleControlData()
        msg.acceleration_pct = 0.6
        msg.braking_pct = 0.0
        
        if self.point_index < self.num_points and self.current_x is not None and self.current_x is not None:
            curr_point = self.check_points[self.point_index]
            end_point = self.check_points[-1]
            
            # update distance to current and previous check point
            curr_point.update(self.current_x, self.current_z)
            end_point.update(self.current_x, self.current_z)
            
            msg.target_wheel_angle = curr_point.get_turn_rad()
            if abs(msg.target_wheel_angle) > RAD_THRESHOLD:
                msg.braking_pct = 0.4
            
            # print(msg.target_wheel_angle, self.point_index, curr_point.get_dis())
            
            if curr_point.get_dis() < DIS_THRESHOLD:
                self.point_index += 1
            
            if end_point.get_dis() < DIS_END_THRESHOLD:
                msg.acceleration_pct = 0.0
                msg.braking_pct = 1.0
        
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
        npc_mask = cv2.inRange(img_hls, (210/2, 0.04*255, 0.70*255), (220/2, 0.12*255, 0.98*255))
        cnts = cv2.findContours(npc_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        self.brake_npc = False
        # print(max([cv2.contourArea(c) for c in cnts]))
        for c in cnts:
            if cv2.contourArea(c) > 6000:
                print(f"Brake: {cv2.contourArea(c)=}")
                self.brake_npc = True
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
