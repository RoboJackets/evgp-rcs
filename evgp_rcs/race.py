from PyQt5.QtCore import pyqtSignal, QObject

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
    DATA_SIZE = 6
    IP = 0
    TEAM = 1
    IS_CONNECTED = 2
    STATE = 3
    LAST_RESPONSE = 4
    ERROR = 5

    HEADER_LABELS = ["IP", "Team", "Connected", "State", "Last Response", "Error"]

    def __init__(self, team, ip_addr):
        self.ip = ip_addr
        self.team = team
        self.is_connected = False
        self.state = RaceState.IN_GARAGE
        self.last_response = RaceState.IN_GARAGE
        self.error = None #TODO: make use of this

    #USED ONLY FOR PYQT DISPLAY
    def index(self, idx):
        return [self.ip, self.team, self.is_connected, self.state.name, self.last_response.name, self.error][idx]

    def set_state(self, state):
        self.state = state
