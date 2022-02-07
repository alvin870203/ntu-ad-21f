import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry  # unit of distance is in meter
from lgsvl_msgs.msg import VehicleControlData, Detection3DArray

import time


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

        # increase timer_period_sec
        self.tmr = self.create_timer(0.01, self.controller_callback)

        # list of x, y, z, time
        self.previous_odom = None
        self.current_odom = None
        self.current_bb = None

        self.to_break = False
        self.speeds = []

    def controller_callback(self):
        # TODO: implement your car controller

        msg = VehicleControlData()

        if self.current_odom is None:
            print("Waiting for the simulator to start")
            msg.acceleration_pct = 0.0
            msg.braking_pct = 0.0
        elif self.to_break:
            print("Break")
            msg.acceleration_pct = 0.0
            msg.braking_pct = 1.0
        else:
            print("Accelerate")
            msg.acceleration_pct = 1.0
            msg.braking_pct = 0.0

        self.pub.publish(msg)

    def bounding_boxes_callback(self, data):
        # TODO: scan neighboring cars
        #print('There are %d objects.' % len(data.detections))

        for index, det in enumerate(data.detections):
            box = det.bbox
            #print('object %d: central of box is at %f %f %f.' %
            #     (index, box.position.position.x, box.position.position.y,
            #      box.position.position.z))  # relative distance
            self.current_bb = [box.position.position.x,
                               box.position.position.y,
                               box.position.position.z,
                               time.time()]

            # conditions for speed control state decision
            too_fast_and_close = (self.current_bb[0] < 25 
                                  and any(speed > 20 for speed in self.speeds))
            too_close_when_slow = (self.current_bb[0] < 7 
                                   and all(speed < 20 for speed in self.speeds))
            too_slow_and_far = (self.current_bb[0] > 5 
                                and all(speed < 15 for speed in self.speeds))

            if self.to_break == False and (too_fast_and_close or too_close_when_slow):
                self.to_break = True
            elif self.to_break == True and too_slow_and_far:
                self.to_break = False
            else:
                self.to_break = self.to_break

            self.speeds = []  # reset speeds log

    def odom_callback(self, data):
        # TODO: store vehicle position
        position = data.pose.pose.position
        #print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        self.previous_odom = self.current_odom
        self.current_odom = [position.x, position.y, position.z, time.time()]
        if self.previous_odom is not None and self.current_odom is not None:
            speed = (((self.current_odom[0] - self.previous_odom[0]) ** 2
                      + (self.current_odom[1] - self.previous_odom[1]) ** 2
                      + (self.current_odom[2] - self.previous_odom[2]) ** 2) ** 0.5
                     / (self.current_odom[3] - self.previous_odom[3]))
            #print(speed)
            self.speeds.append(speed)


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

