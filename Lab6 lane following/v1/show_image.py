import rclpy
from rclpy.node import Node


from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from matplotlib import pyplot as plt
from lane import Lane

class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.create_subscription(CompressedImage, '/simulator/camera_node/image/compressed',
                self.camera_callback, 10)
        self.fig, self.ax = plt.subplots()


    def camera_callback(self, data):
        image_format = data.format
        image_data = np.array(data.data, dtype=np.uint8)
        img = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        print('image format: ', image_format,', size: ',img.shape)


        #TODO
        lane_obj = Lane(orig_frame=img)
        # lane_line_markings = lane_obj.get_line_markings()
        lane_line_markings = lane_obj.my_get_line_markings()
        lane_obj.plot_roi(plot=False)
        warped_frame, warped_plot = lane_obj.perspective_transform(plot=False)
        histogram = lane_obj.calculate_histogram(plot=False)
        try:
            ##left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(plot=False)
            left_fit = lane_obj.get_lane_line_indices_sliding_windows(plot=False)
            ##lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)
            result = lane_obj.get_lane_line_previous_window(left_fit, plot=False)
            ##frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)
            ##self.left_curvem, self.right_curvem = lane_obj.calculate_curvature(print_to_terminal=False)
            self.left_curvem = lane_obj.calculate_curvature(print_to_terminal=False)
            self.center_offset = lane_obj.calculate_car_position(print_to_terminal=False)
            frame_with_lane_lines2 = lane_obj.display_curvature_offset(frame=warped_frame, plot=False)
            cv2.imshow("Image", cv2.resize(warped_plot, (960, 540)))


            # cv2.imwrite('wrong.jpg', img)
            # cv2.imshow('windows', cv2.resize(fame_with_lane_lines2, (960, 540)))
            cv2.waitKey(1)
        except TypeError:
            print("No lane detected")

        #TODO

rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()


