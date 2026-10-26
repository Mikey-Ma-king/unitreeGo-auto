import time
import rclpy
from rclpy.node import Node

import socket

import rclpy.serialization
from unitree_api.msg import Request
from unitree_go.msg import WirelessController


class UDPControllerClient(Node):

    def __init__(self):
        super().__init__('udp_controller_client')
        sport_pub = self.create_publisher(Request, '/api/sport/request', 10)
        time.sleep(0.1)
        req = Request()
        req.header.identity.api_id = 1005
        sport_pub.publish(req)
        self.destroy_publisher(sport_pub)
        self.js_pub = self.create_publisher(WirelessController, '/udp_controller', 10)


if __name__ == '__main__':
    rclpy.init(args=None)

    controller = UDPControllerClient()

    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
    client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    client.bind(("", 37021))

    try:
        while True:
            rclpy.spin_once(controller, timeout_sec=0)
            data, addr = client.recvfrom(128)
            msg = rclpy.serialization.deserialize_message(data, WirelessController)
            controller.js_pub.publish(msg)

    finally:
        controller.destroy_node()
        rclpy.shutdown()
