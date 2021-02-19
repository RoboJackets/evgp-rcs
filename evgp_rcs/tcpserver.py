from PyQt5.QtCore import QObject, QThread, pyqtSignal
import socket
import select
from race import RaceState


class TcpServer(QObject):

    new_connection = pyqtSignal(str) #ip as string
    lost_connection = pyqtSignal(str) #ip as string
    new_response = pyqtSignal(str, RaceState) #ip of responder as string, response as RaceState

    def __init__(self, server_port, server_backlog):
        self.server_port = server_port
        self.server_backlog = server_backlog
        self.continue_run = True
        self.race_state = RaceState.IN_GARAGE
        self.connections = []


    def stop(self):
        self.continue_run = False

    @QtCore.pyqtSlot(RaceState)
    def on_race_state_change(self, newState):
        self.race_state = newState

    def start_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setblocking(0)
        self.server.bind((socket.gethostname(), self.server_port))
        self.server.listen(self.server_backlog)

    def run_server(self):
        while self.continue_run:
            inputs = self.connections.copy().append(self.server)
            readable, writable, exceptional = select.select(
                inputs, self.connections, inputs)
            for s in readable:
                if s is self.server:
                    connection, client_address = s.accept()
                    connection.setblocking(0)
                    self.connections.append(connection)
                    new_connection.emit(client_address)
                    print(f"accepting new client at {client_address}") #TODO: logging
                else:
                    data = s.recv(1024)
                    if data:
                        pass
                        #TODO: process
                    else:
                        self.connection.remove(s)
                        s.close()
                        self.race.removeRacer()
            for s in writable:
                #TODO: some time processing so we don't send repeat signal too fast
                pass
