from PyQt5.QtCore import QObject, QThread, pyqtSignal
import socket
import select
import sys
import re
import time
import logging
from race import RaceState


class TCPServer(QObject):

    START_CHAR = '$'
    END_CHAR = ';'

    new_connection = pyqtSignal(str) #ip as string
    lost_connection = pyqtSignal(str) #ip as string
    new_response = pyqtSignal(str, RaceState) #ip of responder as string, response as RaceState

    server_ready = pyqtSignal(bool)

    def __init__(self, server_port, server_backlog=10, whitelist=None, parent=None):
        QObject.__init__(self, parent=parent)
        self.server_port = server_port
        self.server_backlog = server_backlog
        self.continue_run = True
        self.connections = []
        self.states = {}
        self.responses = {}
        self.leftover_messages = {}
        self.connection_to_addr = {} #TODO: may need to run off (addr,port) or handle oddity if car tries to connect twice
        self.whitelist = whitelist
        if self.whitelist is None:
            self.whitelist = []

    def stop(self):
        logging.info("Requesting server shutdown")
        self.continue_run = False #no mutex don't worry about it

    def on_race_state_change(self, ip, newState): #TODO: check speed of handling a "go" as individual events
        self.states[ip] = newState
        logging.debug(f"{ip} state changed to {newState}")

    #returns True on success, false otherwise
    def start_server(self):
        tries = 5
        backoff = 5 #sec
        for i in range(tries):
            try:
                self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server.setblocking(0)
                self.server.bind(("0.0.0.0", self.server_port))
                self.server.listen(self.server_backlog)
                logging.info(f"Server starting on port {self.server_port}. Look up your LAN IP address to connect.")
                self.server_ready.emit(True)
                return True
            except Exception as e:
                if i < tries - 1:
                    logging.error(f"Server failed to start. Retrying in {backoff} seconds. Exception was: {e}")
                    time.sleep(backoff)
                    continue
                else:
                    logging.critical("Server is errored. Please restart program and try again. Exception was: {e}")
                    self.server_ready.emit(False)
                    return False

    def close_server(self):
        for s in self.connections:
            s.shutdown(socket.SHUT_RDWR)
            s.close()
        self.connections = []
        self.state = {}
        self.connection_to_addr = {}
        self.server.shutdown(socket.SHUT_RDWR)
        self.server.close()
        logging.info("Server shut down by user request")

    def remove_lost_client(self, client):
        addr = self.connection_to_addr[client]
        self.connections.remove(client)
        del self.states[addr]
        del self.responses[addr]
        self.lost_connection.emit(addr)
        client.close()
        logging.debug(f"Closed client connection to {addr}")

    def run_server(self):
        ret = self.start_server()
        if not ret:
            return
        while self.continue_run:
            inputs = self.connections.copy()
            inputs.append(self.server)
            readable, writable, exceptional = select.select(
                inputs, self.connections, inputs, 0)
            for s in readable:
                if s is self.server:
                    #NEW CONNECTION
                    connection, client_address = s.accept()
                    connection.setblocking(0)
                    if client_address[0] not in self.whitelist:
                        logging.debug(f"Ignoring unknown connection from {client_address}. Not in whitelist!")
                        connection.close() # we don't know you!
                        continue
                    self.connections.append(connection)
                    self.connection_to_addr[connection] = client_address[0]
                    self.states[client_address[0]] = RaceState.IN_GARAGE
                    self.responses[client_address[0]] = RaceState.IN_GARAGE
                    self.leftover_messages[client_address[0]] = ""
                    logging.debug(f"Accepting new client at {client_address[0]} port {client_address[1]}") #TODO: logging
                    self.new_connection.emit(client_address[0]) #TODO: handle double IP connections just in case
                else:
                    try:
                        data = s.recv(1024)
                        if data:
                            addr, port = s.getpeername()
                            msg = data.decode('utf-8')
                            self.process_message(addr, msg)
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
        self.close_server()


    def process_message(self, ip, msg):
        #Grabs the last message only. Teams are expected to be consistent.
        all_message_data = self.leftover_messages[ip] + msg
        matches = re.findall('\$([^\$\s]+?);', all_message_data) #splits "$something;" to "something"

        last_start = max(all_message_data.rfind(self.START_CHAR),0)
        last_end = max(all_message_data.rfind(self.END_CHAR),0)
        if (last_end > last_start):
            self.leftover_messages[ip] = ""
        else:
            self.leftover_messages[ip] = all_message_data[last_start:]

        if matches:
            last_msg = matches[-1]
            try:
                new_state = RaceState(last_msg)
                last_state = self.responses[ip]
                if new_state != last_state: #only report changes to RCSModel
                    self.new_response.emit(ip, new_state)
                    self.responses[ip] = new_state
            except ValueError as e:
                logging.error(e)
                return

# Use to debug without GUI
# server = TcpServer(8080, 10)
# server.start_server()
# server.run_server()
