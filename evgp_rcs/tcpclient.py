import socket
import sys
import signal
import re
import time

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

start = time.time()
split = 5
states = ["$IN_GARAGE;", "$GREEN_GREEN;", "$RED_RED;"]
selection = 0

leftover_message = ""

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
            all_message_data = leftover_message + data.decode('utf-8')
            matches = re.findall('\$([^\$\s]+?);', all_message_data) #splits "$something;"" to "something"
            last_end = max(all_message_data.rfind(";"),0)
            last_start = max(all_message_data.rfind("$"),0)
            if (last_end > last_start):
                leftover_message = ""
            else:
                leftover_message = all_message_data[last_start:]

            if not matches:
                print(leftover_message)
                print(data.decode('utf-8')) #TODO: keep track of data that wasn't matched
            else:
                msg = matches[-1]
                #print(msg)
                if msg != last_msg:
                    print(f"received {msg}")
                    last_msg = msg

        # send test messsages
        # if time.time() - start > split:
        #     message = states[selection%len(states)]
        #     selection += 1
        #     print(f'sending {message}')
        #     sock.sendall(message.encode('utf-8'))
        #     start = time.time()

finally:
    print("closing socket")
    sock.close()
