import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from lgsvl_msgs.msg import VehicleControlData, Detection3DArray


class Driver(Node):
    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(VehicleControlData,
                                         '/lgsvl/vehicle_control_cmd', 10)
        self.create_subscription(Detection3DArray,
                                 '/ground_truth/bounding_boxes',
                                 self.bounding_boxes_callback, 10)
        self.create_subscription(Odometry, '/lgsvl/gnss_odom',
                                 self.odom_callback, 10)

        self.tmr = self.create_timer(1.0, self.controller_callback)

    def controller_callback(self):
        # TODO: implement your car controller

        msg = VehicleControlData()
        msg.acceleration_pct = 0.0
        msg.braking_pct = 1.0

        self.pub.publish(msg)

    def bounding_boxes_callback(self, data):
        # TODO: scan neighboring cars
        print('There are %d objects.' % len(data.detections))

        for index, det in enumerate(data.detections):
            box = det.bbox
            print('object %d: central of box is at %f %f %f.' %
                  (index, box.position.position.x, box.position.position.y,
                   box.position.position.z))

    def odom_callback(self, data):
        # TODO: store vehicle position
        position = data.pose.pose.position
        print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))


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
