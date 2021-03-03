import socket
import sys
import signal


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



state = "$GREEN_GREEN;"

try:
    # Send data
    message = "$GREEN_GREEN;"
    print(f'sending {message}')
    sock.sendall(message.encode('utf-8'))
    while True:

        data = sock.recv(256)
        amount_received = len(data)
        if amount_received > 0:
            print(f"received {data.decode('utf-8')}")

finally:
    print("closing socket")
    sock.close()
