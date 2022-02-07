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
                                 self.bounding_boxes_callback, 1000)
        self.create_subscription(Odometry, '/lgsvl/gnss_odom',
                                 self.odom_callback, 1000)

        self.tmr = self.create_timer(0.5, self.controller_callback)  # increase timer_period_sec
        
        # list of x, y, z
        self.current_odom = None
        self.current_bb = None
        
        self.threshold = 8  # threshold is set by experience
        self.to_break = False  # if (odom x - y) < threshold, then start breaking
        
    def controller_callback(self):
        # TODO: implement your car controller

        msg = VehicleControlData()
        
        if self.current_odom is None:
        	print("Waiting simulator")
        	msg.acceleration_pct = 0.0
        	msg.braking_pct = 0.0
        elif self.current_bb is None and not self.to_break:
        	print("Accelerate")
        	#print(self.current_odom[0] - self.current_odom[1])
        	msg.acceleration_pct = 1.0
        	msg.braking_pct = 0.0
        else:
        	print("Break")
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
            self.current_bb = [box.position.position.x,
            				   box.position.position.y,
            				   box.position.position.z]

    def odom_callback(self, data):
        # TODO: store vehicle position
        position = data.pose.pose.position
        print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        self.current_odom = [position.x, position.y, position.z]
        #print(position.x - position.y)
        if (position.x - position.y) < self.threshold:
        	self.to_break = True


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
