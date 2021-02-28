
from enum import Enum
class RaceState(Enum):
    IN_GARAGE = "IN_GARAGE"
    GRID_ACTIVE = "GRID_ACTIVE"
    GREEN_GREEN = "GREEN_GREEN"
    RED_FLAG = "RED_FLAG"
    RED_RED = "RED_RED"
    
    def __str__(self):
        return str(self.value)


class Racer:
    #USED ONLY FOR PYQT DISPLAY
    DATA_SIZE = 5 #TODO: not this?
    IP = 0
    TEAM = 1
    STATE = 2
    LAST_RESPONSE = 3
    IS_ERRORED = 4 #TODO: better define by disconnection

    def __init__(self, team, ip_addr, socket):
        self.ip = ip_addr
        self.team = team
        self.socket = socket
        self.state = RaceState.IN_GARAGE
        self.last_response = RaceState.IN_GARAGE
        self.is_errored = False

    #USED ONLY FOR PYQT DISPLAY
    def index(self, idx):
        return [self.ip, self.team, self.state.name, self.last_response.name, self.is_errored][idx]

    def set_state(self, state):
        self.state = state
        #TODO: EMIT SIGNAL for TCP server?

class Race:
    def __init__(self):
        self.racers = []
        self.state = RaceState.IN_GARAGE

    def get_racer_count(self):
        return len(self.racers)

    def get_racer(self, index):
        return self.racers[index]

    def createNewRacer(self, team, connection, client_address):
        r = Racer(team, client_address, connection)
        self.racers.append(r)

    def addRacer(self, newRacer):
        self.racers.append(newRacer)

    def removeRacer(self, index):
        return self.racers.pop(index)

    def race_state_change(self, state):
        #TODO: some logic around ensuring proper state flow
        for i in range(len(self.racers)):
            self.state = state
            self.racers[i].set_state(state)
