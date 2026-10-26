import rclpy
from rclpy.node import Node

import socket

import rclpy.serialization
from unitree_go.msg import WirelessController
import subprocess
import re

# Run the ip command and capture its output
result = subprocess.run(['ip', 'a', 'show', 'dev', 'wlan0'], capture_output=True, text=True)
output = result.stdout
# Define regular expressions to match the IP and broadcast addresses
ip_regex = re.compile(r'inet (\d+\.\d+\.\d+\.\d+)/\d+ brd (\d+\.\d+\.\d+\.\d+)')

# Search for matches in the output
matches = ip_regex.search(output)
if matches:    
    ip_address = matches.group(1)
    brd_address = matches.group(2)

DST_IP = brd_address

class UDPControllerServer(Node):

    def __init__(self):
        super().__init__('udp_controller_server')
        self.js_sub = self.create_subscription(WirelessController, '/wirelesscontroller', self.js_callback, 10)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.server.settimeout(0.2)

    def js_callback(self, msg:WirelessController):
        data = rclpy.serialization.serialize_message(msg)
        self.server.sendto(data, (DST_IP, 37021))


if __name__ == '__main__':
    rclpy.init(args=None)
    controller = UDPControllerServer()
    rclpy.spin(controller)
