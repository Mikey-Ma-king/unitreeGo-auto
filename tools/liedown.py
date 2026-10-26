import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

class DpNavigation(Node):

    def __init__(self):
        super().__init__('liedown')
        self.sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        time.sleep(0.1)
        req = Request()
        req.header.identity.api_id = 1005
        self.sport_pub.publish(req)

def main(args=None):
    rclpy.init(args=args)
    planner = DpNavigation()
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
