import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class DpNavigation(Node):

    def __init__(self):
        super().__init__('dp_navigation')
        self.cloud_sub = self.create_subscription(PointCloud2, '/utlidar/cloud', self.cloud_callback, 10)

    def cloud_callback(self, data: PointCloud2):
        for ptc in point_cloud2.read_points(data, field_names=("x", "y", "z")):
            breakpoint()
        print(data)


rclpy.init()
planner = DpNavigation()
rclpy.spin(planner)
