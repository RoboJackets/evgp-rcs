from PyQt5.QtCore import QObject, QThread, pyqtSignal
import socket
import select
import sys
import re
from race import RaceState


class TCPServer(QObject):

    START_CHAR = '$'
    END_CHAR = ';'

    new_connection = pyqtSignal(str) #ip as string
    lost_connection = pyqtSignal(str) #ip as string
    new_response = pyqtSignal(str, RaceState) #ip of responder as string, response as RaceState

    def __init__(self, server_port, server_backlog=10, parent=None):
        QObject.__init__(self, parent=parent)
        self.server_port = server_port
        self.server_backlog = server_backlog
        self.continue_run = True
        self.connections = []
        self.states = {}
        self.responses = {} #probably don't need after we report the response to the model
        self.connection_to_addr = {} #TODO: may need to run off (addr,port) or handle oddity if car tries to connect twice


    def stop(self):
        print("Requesting server shutdown")
        self.continue_run = False

    def on_race_state_change(self, ip, newState): #TODO: check speed of handling a "go" as individual events
        self.states[ip] = newState
        print(f"{ip} state changed to {newState}")

    def start_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setblocking(0)
        self.server.bind(("0.0.0.0", self.server_port))
        self.server.listen(self.server_backlog)
        print(f"Server starting at 0.0.0.0 (all local addresses) on port {self.server_port}")

    def close_server(self):
        for s in self.connections:
            s.shutdown(socket.SHUT_RDWR)
            s.close()
        self.connections = []
        self.state = {}
        self.connection_to_addr = {}
        self.server.shutdown(socket.SHUT_RDWR)
        self.server.close()
        print("Server shut down by user request")

    def remove_lost_client(self, client):
        addr = self.connection_to_addr[client]
        self.connections.remove(client)
        del self.states[addr]
        del self.responses[addr]
        self.lost_connection.emit(addr)
        print(f"Closed client connection to {addr}")

    def run_server(self):
        # try:
        self.start_server()
        while self.continue_run:
            inputs = self.connections.copy()
            inputs.append(self.server)
            readable, writable, exceptional = select.select(
                inputs, self.connections, inputs)
            for s in readable:
                if s is self.server:
                    #NEW CONNECTION
                    connection, client_address = s.accept()
                    connection.setblocking(0)
                    self.connections.append(connection)
                    self.connection_to_addr[connection] = client_address[0]
                    self.states[client_address[0]] = RaceState.IN_GARAGE
                    self.responses[client_address[0]] = RaceState.IN_GARAGE
                    print(f"accepting new client at {client_address[0]} port {client_address[1]}") #TODO: logging
                    self.new_connection.emit(client_address[0]) #TODO: handle unknown connections just in case
                else:
                    try:
                        data = s.recv(1024)
                        if data:
                            addr, port = s.getpeername()
                            msg = data.decode('utf-8')
                            self.process_message(addr, msg)
                        else:
                            pass #TODO: something
                    except OSError:
                        self.remove_lost_client(s)
                        try:
                            writable.remove(s)
                        except ValueError:
                            pass

            for s in writable:
                #TODO: some time processing so we don't send repeat signal too fast
                try:
                    (addr, port) = s.getpeername()
                    state = self.states[addr]
                    msg = self.START_CHAR + str(state) + self.END_CHAR
                    s.send(msg.encode('utf-8'))
                except OSError:
                    self.remove_lost_client(s)
            #TODO: handle exceptional?
        self.close_server()
        # except: #TODO: better handling of shutdown?
        #     print("Unexpected error:", sys.exc_info()[0])
        #     self.close_server()


    def process_message(self, ip, msg):
        #TODO: handle if a full message doesn't come at once, saving things after ; for next time around?
        #TODO: handle multiple messages at once -- I think just grab the last?
        matches = re.findall('\$([^\$\s]+?);', msg) #splits "$something;"" to "something"

        if matches: #TODO: report error if effectively empty message?
            last_msg = matches[-1]
            try:
                new_state = RaceState(last_msg)
                last_state = self.states[ip]
                if new_state != last_state: #only report changes to RCSModel
                    self.new_response.emit(ip, new_state)
                    self.responses[ip] = new_state
            except ValueError as e:
                print(e) #TODO: logging of oddity
                return

#TODO: remove this debugging
# server = TcpServer(8080, 10)
# server.start_server()
# server.run_server()
