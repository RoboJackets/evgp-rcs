import socket
import sys
import signal
import re


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    done = True
    sock.close()
    sys.exit(0)

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = (socket.gethostname(), 8080)
print('connecting to %s port %s' % server_address)
sock.connect(server_address)



state = "$IN_GARAGE;"#"$GREEN_GREEN;"

try:
    # Send data
    last_msg = None
    message = state
    print(f'sending {message}')
    sock.sendall(message.encode('utf-8'))
    while True:

        data = sock.recv(256)
        amount_received = len(data)
        if amount_received > 0:
            matches = re.findall('\$([^\$\s]+?);', data.decode('utf-8')) #splits "$something;"" to "something"
            if not matches:
                print(data.decode('utf-8'))
            else:
                msg = matches[-1]
                if msg != last_msg:
                    print(f"received {msg}")
                    last_msg = msg

finally:
    print("closing socket")
    sock.close()
