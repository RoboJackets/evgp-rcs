from PyQt5.QtCore import QObject, QThread, pyqtSignal
import socket
import select
import re
from race import RaceState


class TcpServer(QObject):

    START_CHAR = '$'
    END_CHAR = ';'

    new_connection = pyqtSignal(str) #ip as string
    lost_connection = pyqtSignal(str) #ip as string
    new_response = pyqtSignal(str, RaceState) #ip of responder as string, response as RaceState

    def __init__(self, server_port, server_backlog):
        self.server_port = server_port
        self.server_backlog = server_backlog
        self.continue_run = True
        self.connections = []
        self.states = {}


    def stop(self):
        self.continue_run = False

    @QtCore.pyqtSlot(RaceState)
    def on_race_state_change(self, ip, newState): #TODO: check speed of handling a "go" as individual events
        self.states[ip] = newState

    def start_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setblocking(0)
        self.server.bind((socket.gethostname(), self.server_port))
        self.server.listen(self.server_backlog)

    def close_server(self):
        for s in self.connections:
            s.shutdown()
            s.close()
        s = []
        self.server.close()

    def run_server(self):
        while self.continue_run:
            inputs = self.connections.copy().append(self.server)
            readable, writable, exceptional = select.select(
                inputs, self.connections, inputs)
            for s in readable:
                if s is self.server:
                    #NEW CONNECTION
                    connection, client_address = s.accept()
                    connection.setblocking(0)
                    self.connections.append(connection)
                    new_connection.emit(client_address)
                    print(f"accepting new client at {client_address}") #TODO: logging
                else:
                    data = s.recv(1024)
                    if data:
                        addr = s.getpeername()
                        self.process_message(addr, msg)
                    else:
                        #LOST A CONNECTION
                        self.connection.remove(s)
                        addr = s.getpeername()
                        s.shutdown()
                        s.close()
                        del self.states[addr]
                        lost_connection.emit(addr)
                        print(f"Server lost connection to {addr}")

            for s in writable:
                #TODO: some time processing so we don't send repeat signal too fast
                addr = s.getpeername()
                print(addr) #TODO: rm debug
                state = self.states[addr]
                msg = START_CHAR + str(state) + END_CHAR
                s.send(msg)
            #TODO: handle exceptional?
        self.close_server()


    def process_message(self, ip, msg):
        #TODO: handle if a full message doesn't come at once ("unlikely")
        #TODO: handle multiple messages at once -- I think just grab the last?
        matches = re.findall('\$([^\$\s]+?);', a) #splits "$something;"" to "something"

        if matches: #TODO: report error if effectively empty message?
            last_msg = matches[-1]
            try:
                new_state = RaceState(last_msg)
                last_state = self.states[ip]
                if new_state != last_state: #only report changes to RCSModel
                    new_response.emit(ip, new_state)
                    self.states[ip] = new_state
            except ValueError as e:
                print(e) #TODO: logging of oddity
                return
