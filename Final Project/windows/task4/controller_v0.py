from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from lgsvl_msgs.msg import VehicleControlData
import math

DIS_THRESHOLD = 3.0  # TODO: tune this to stop at right last point
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

        sparse_waypoints = [
            # [-67.1278076171875 ,-1.16648268699646],
            # [-66.7058639526367 ,-2.32602906227112],
            # [-66.2109756469727 ,-3.62944030761719],
            # [-65.6610260009766 ,-4.72425031661987],
            # [-64.7497253417969 ,-6.01844215393066],
            # [-63.8709106445313 ,-6.88123559951782],
            # [-62.6199340820313 ,-7.90589618682861],
            # [-61.4793472290039 ,-8.70077800750732],
            # [-59.9385910034180 ,-9.66747856140137],
            [-64.88, -6.84],
            [-58.560997, -9.557620],
            # [-58.4949874877930 ,-10.2915391921997],
            # [-57.6773147583008 ,-10.4582862854004],
            # [-57.1246948242188 ,-10.4347305297852],
            # [-56.6399574279785 ,-10.3712787628174],
            # [-56.2219047546387 ,-10.2361173629761],
            # [-55.7975769042969 ,-10.1123495101929],
            # [-55.3906135559082 ,-9.91606616973877],
            # [-54.9307823181152 ,-9.67999744415283],
            [-54.556374, -9.354400],
            # [-54.4204521179199 ,-9.37167930603027],
            # [-53.8204193115234 ,-9.04186725616455],
            # [-53.1503295898438 ,-8.68472766876221],
            # [-52.1685600280762 ,-8.20993804931641],
            # [-51.2691230773926 ,-7.80717229843140],
            # [-50.2680397033691 ,-7.41438531875610],
            # [-49.1685409545898 ,-7.02960205078125],
            # [-47.9609222412109 ,-6.66412973403931],
            # [-46.6546440124512 ,-6.26883506774902],
            [-42.16 ,-4.9]
        ]

        # load waypoints
        self.check_points = []
        # with open("waypoints.csv") as fp:
        #     for line in fp:
        #         waypoint_x, waypoint_z = [float(v) for v in line.split(",")]
        #         self.check_points.append(CheckPoint(waypoint_x, waypoint_z))
        for waypoint_x, waypoint_z in sparse_waypoints:
            self.check_points.append(CheckPoint(waypoint_x, waypoint_z))

        # self.check_points.append(CheckPoint(78.77, -11.41))  # End point
        self.num_points = len(self.check_points)
        assert self.num_points > 0
        
        self.tmr = self.create_timer(0.11, self.controller_callback)
        
        # store current position_callback
        self.current_x = None
        self.current_z = None
        
        self.point_index = 0

    def controller_callback(self):
        # TODO: implement your car controller

        msg = VehicleControlData()
        msg.acceleration_pct = 0.15
        msg.braking_pct = 0.0
        
        if self.point_index < self.num_points and self.current_x is not None and self.current_x is not None:
            curr_point = self.check_points[self.point_index]
            
            # update distance to current and previous check point
            curr_point.update(self.current_x, self.current_z)
            
            msg.target_wheel_angle = curr_point.get_turn_rad()
            if abs(msg.target_wheel_angle) > RAD_THRESHOLD:
                msg.braking_pct = 0.0#0.15
            
            # print(msg.target_wheel_angle, self.point_index, curr_point.get_dis())
            
            if curr_point.get_dis() < DIS_THRESHOLD:
                self.point_index += 1
        
        elif self.point_index == self.num_points:  # last point, stop
            msg.acceleration_pct = 0.0
            msg.braking_pct = 1.0
        
        self.pub.publish(msg)

    def position_callback(self, data):
        # TODO: store vehicle position
        self.current_x = data.x
        self.current_z = data.z
        print("Current pos: %f, %f, %f" % (data.x, data.y, data.z))


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
