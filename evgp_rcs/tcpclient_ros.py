import socket
import sys
import signal
import re
import time
import select
import argparse
import platform

from threading import Thread, Lock

import rospy
from std_msgs.msg import Bool, Float64

from race import RaceState


class TCPClient:

    START_CHAR = '$'
    END_CHAR = ';'

    def __init__(self, server_ip, server_port):
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        server_address = (server_ip, server_port)
        print('connecting to %s port %s' % server_address)
        self.sock.connect(server_address)

        self.last_msg = ""
        self.leftover_message = ""

    def receive_message(self):
        data = self.sock.recv(256)
        amount_received = len(data)
        if amount_received > 0:
            all_message_data = self.leftover_message + data.decode('utf-8')
            matches = re.findall('\$([^\$\s]+?);', all_message_data) #splits "$something;"" to "something"
            last_end = max(all_message_data.rfind(self.END_CHAR),0)
            last_start = max(all_message_data.rfind(self.START_CHAR),0)
            if (last_end > last_start):
                self.leftover_message = ""
            else:
                self.leftover_message = all_message_data[last_start:]

            if not matches:
                #print(data.decode('utf-8'))
                pass
            else:
                msg = matches[-1]
                if msg != self.last_msg:
                    print(f"received {msg}")
                    self.last_msg = msg
                    return msg

        return None

    def send_message(self, msg):
        if msg is not None:
            msg = f"{self.START_CHAR}{msg}{self.END_CHAR}"
            self.sock.sendall(msg.encode('utf-8'))
            print(f"sent {msg}")

    def close(self):
        self.sock.close()



class RaceController:

    def __init__(self):
        rospy.init_node('rcs_client', anonymous=True)

        server_ip = socket.gethostname()
        server_port = 12018
        delay = 1

        self.tcpclient = TCPClient(server_ip, server_port)

        self.is_ready = True
        self.cmd_speed = Float64()
        self.cmd_steering = Float64()

        rospy.Subscriber("ready", Bool, self.ready_callback)

        rospy.Subscriber("/planned/speed", Float64, self.plan_speed_callback)
        rospy.Subscriber("/planned/steering", Float64, self.plan_steering_callback)

        self.speed_pub = rospy.Publisher('/speed', Float64)
        self.steering_pub = rospy.Publisher('/steering', Float64)

        self.msg_from_server = None

        self.server_listener_thr = Thread(target=self.server_listener)
        self.server_listener_thr.start()

        self.control_loop_thr = Thread(target=self.control_loop)
        self.control_loop_thr.start()

    def kill(self, signal, frame):
        print('You pressed Ctrl+C! Closing TCPClient, threads, and exiting.')
        self.tcpclient.close()
        self.server_listener_thr.join()
        self.control_loop_thr.join()
        sys.exit(0)


    def server_listener(self):
        # publish_rate = rospy.Rate(100)
        while True:
            msg_from_server = self.tcpclient.receive_message()
            if msg_from_server:
                self.msg_from_server = msg_from_server
            # publish_rate.sleep()
            # TODO: Mutex?


    def control_loop(self):
        control_rate = rospy.Rate(50)
        publish_rate = rospy.Rate(1)
        last_msg_sent = None

        while True:
            msg_from_server = self.msg_from_server
            msg = None

            if RaceState.IN_GARAGE == msg_from_server and last_msg_sent is None:
                msg = RaceState.IN_GARAGE

            elif RaceState.GRID_ACTIVE == msg_from_server and self.is_ready:
                # TODO: Reset Possible?
                msg = RaceState.GRID_ACTIVE

            elif RaceState.GREEN_GREEN == msg_from_server:
                # Moving (Passthrough speed and steering)
                msg = RaceState.GREEN_GREEN
                self.publish_command(speed=self.cmd_speed, steering=self.cmd_steering)

            elif RaceState.RED_FLAG == msg_from_server:
                # Slow Stop (Passthrough only steering)
                msg = RaceState.RED_FLAG
                self.publish_command(speed=Float64(0), steering=self.cmd_steering)

            elif RaceState.RED_RED == msg_from_server:
                # Hard Stop
                msg = RaceState.RED_RED
                self.publish_command(speed=Float64(0), steering=Float64(0))

            if msg and msg != last_msg_sent:
                publish_rate.sleep() # If < 1 sec passed since last pub, sleep
                self.tcpclient.send_message(msg)
                last_msg_sent = msg

            control_rate.sleep()

    def ready_callback(self, bool_msg):
        self.is_ready = bool_msg.data

    def plan_speed_callback(self, cmd_speed_msg):
        self.cmd_speed = cmd_speed_msg

    def plan_steering_callback(self, cmd_steering_msg):
        self.cmd_steering = cmd_steering_msg

    def publish_command(self, speed, steering):
        self.speed_pub.publish(speed)
        self.steering_pub.publish(steering)



if __name__ == "__main__":
    race_controller = RaceController()
    signal.signal(signal.SIGINT, race_controller.kill)
    rospy.spin()
