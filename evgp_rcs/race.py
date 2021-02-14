
from enum import Enum
class RaceState(Enum):
    IN_GARAGE = "IN_GARAGE"
    GRID_ACTIVE = "GRID_ACTIVE"
    GREEN_GREEN = "GREEN_GREEN"
    RED_FLAG = "RED_FLAG"
    RED_RED = "RED_RED"


class Racer:
    DATA_SIZE = 6 #TODO: not this?

    def __init__(self, team, ip_addr, socket):
        self.ip = ip_addr
        self.team = team
        self.socket = socket
        self.state = RaceState.IN_GARAGE
        self.last_response = RaceState.IN_GARAGE
        self.is_errored = False

    def index(self, idx):
        return [self.ip, self.team, self.socket, self.state, self.last_response, self.is_errored][idx]


class Race:
    def __init__(self):
        self.racers = []
        self.state = RaceState.IN_GARAGE

    def get_racer_count(self):
        return len(self.racers)

    def get_racer(self, index):
        return self.racers[index]

    def addRacer(self, team, connection, client_address):
        r = Racer(team, client_address, connection)
        self.racers.append(r)
        print("Accepted connection from {0} at {1}".format(team, client_address))
