from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from lgsvl_msgs.msg import VehicleControlData


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

        self.tmr = self.create_timer(1.0, self.controller_callback)

    def controller_callback(self):
        # TODO: implement your car controller

        msg = VehicleControlData()
        msg.acceleration_pct = 0.0
        msg.braking_pct = 0.0

        self.pub.publish(msg)

    def position_callback(self, data):
        # TODO: store vehicle position
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
