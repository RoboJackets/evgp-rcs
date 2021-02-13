import socket
import select
import yaml

class RCSModel:

    def __init__(self):
        self.race = Race()
        self.standby = Race()
        self.teams_list = {}

        # self.server = None
        # self.SERVER_PORT = 80
        # self.SERVER_BACKLOG = 10

        def load_team_list(self):
            #TODO: load a map from text file of team name to IP
            self.teams_list = {"team1": "123.123.123.123"}

        # def start_server(self):
        #     self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #     self.server.setblocking(0)
        #     self.server.bind((socket.gethostname(), self.SERVER_PORT))
        #     self.server.listen(self.SERVER_BACKLOG)
        #
        #
        # def run_server(self):
        #     while True:
        #         inputs = [x for x in self.race.racers.socket]
        #         outputs = inputs
        #         inputs.append(self.server)
        #         readable, writable, exceptional = select.select(
        #             inputs, outputs, inputs)
        #         for s in readable:
        #             if s is server:
        #                 connection, client_address = s.accept()
        #                 connection.setblocking(0)
        #                 team = self.teams_list[client_address]
        #                 #TODO: signal/slot for new racer
        #                 self.standby.addRacer(team, connection, client_address)
        #             else:
        #                 data = s.recv(1024)
        #                 if data:
        #                     message_queues[s].put(data)
        #                     if s not in outputs:
        #                         outputs.append(s)
        #                 else:
        #                     if s in outputs:
        #                         outputs.remove(s)
        #                     inputs.remove(s)
        #                     s.close()
        #                     self.race.removeRacer()
