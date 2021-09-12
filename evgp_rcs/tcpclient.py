import socket
import sys
import signal
import re
import time
import select
import argparse
import platform

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

        self.last_receive_time = time.time()

    def receive_message(self):
        data = self.sock .recv(256)
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
                current_time = time.time()
                self.time_between_messages = current_time - self.last_receive_time
                self.last_receive_time = current_time
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

    def get_receive_hz(self):
        return 1.0 / self.time_between_messages

    def close(self):
        self.sock.close()



def signal_handler(signal, frame):
    print('You pressed Ctrl+C! Closing TCPClient and exiting.')
    done = True
    tcpclient.close()
    sys.exit(0)

if __name__ == "__main__":

    # TCP Client Example:
    # well-behaved: Responds with state server requested
    # interval: Iterates through RaceStates and sends the next one (please prove a --delay)
    # single-message: Sends the message from the --message argument
    # interactive: Use 1-5 to select a RaceState to send
    parser = argparse.ArgumentParser(description='Example TCPClient for EVGP RCS.')
    parser.add_argument('--type',
                        default='well-behaved',
                        const='all',
                        nargs='?',
                        choices=['well-behaved', 'interval', 'single-message', 'interactive', 'listen'],
                        help='Type of client.')
    parser.add_argument('--delay', type=float, default=0, help='Delay between messages')
    parser.add_argument('--message', type=str, default="IN_GARAGE", help='Message sent (only when type=single-message)')
    parser.add_argument('--server', type=str, default=socket.gethostname(), help='RCS Server if not on local machine')
    parser.add_argument('--port', type=int, default=12017, help='Port if not RCS default')

    args = parser.parse_args()
    signal.signal(signal.SIGINT, signal_handler)

    states = [e.value for e in RaceState]
    index = 0
    interactive_message = "Use 1-5 to send a state.\n" + "\n".join([f"{i+1}: {s}" for i,s in enumerate(states)])

    tcpclient = TCPClient(args.server, args.port)

    if args.type == 'single-message':
        if args.message:
            time.sleep(args.delay)
            tcpclient.send_message(args.message)
        else:
            print("No message argument provided")
        sys.exit(0)
    elif args.type == "interactive":
        if platform.system() == 'Windows':
            print("Sorry, you can't use interactive mode on Windows.")
        print(interactive_message)
    else:
        pass

    while True:
        msg_from_server = tcpclient.receive_message()

        msg = None
        if args.type == "well-behaved":
            msg = msg_from_server
        elif args.type == "interval":
            msg = states[index]
            index = (index + 1) % len(states)
        elif args.type == "interactive":
            # use 1-5 input to send state message
            # @Note: Select for user input only works on Unix system (not Windows)
            readable, writable, exceptional = select.select([sys.stdin], [], [], 0)
            if sys.stdin in readable:
                line = sys.stdin.readline()
                if line:
                    try:
                        num = int(line) - 1
                        msg = states[num%len(states)]
                        print(interactive_message)
                    except:
                        pass
        else:
            pass

        if msg is not None:
            time.sleep(args.delay)
            tcpclient.send_message(msg)
