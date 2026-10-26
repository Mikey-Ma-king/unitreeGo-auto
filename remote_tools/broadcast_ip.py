import socket
import subprocess
import re
import os
import time

time.sleep(10)

robot_id = os.environ.get("ROBOT_ID", "-1")

while True:
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

        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        server.sendto(robot_id.encode(), (brd_address, 37022))
        time.sleep(5)
        
        print(ip_address)
        print('send successfully')
    else:
        print("No IP or broadcast address found.")
        

