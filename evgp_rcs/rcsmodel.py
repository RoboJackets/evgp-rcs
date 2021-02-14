import socket
import select
import yaml
from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from race import Race, Racer


class RCSModel(QtCore.QAbstractTableModel):

    def __init__(self):
        super(RCSModel, self).__init__()
        self.active_race = Race()
        self.standby_race = Race()
        #TODO: disconnected or not connected racers from teams list should show
        self.teams_list = {}

    def load_team_list(self):
        #TODO: load a map from yaml file of team name to IP
        self.teams_list = {
            "team1": "123.123.123.123",
            "team2": "456.456.456.456",
            "team3": "789.789.789.789"
            }

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if role == Qt.DisplayRole:
            # data is tabled as rows active followed by standy
            # rows are filled with racer data (see race.py definition)
            r = index.row()
            c = index.column()
            if (r < self.active_race.get_racer_count()):
                #active racers
                return self.active_race.get_racer(r).index(c)
            else:
                return self.standby_race.get_racer(r - self.active_race.get_racer_count()).index(c)

    def rowCount(self, parent=QtCore.QModelIndex()):
        return self.active_race.get_racer_count() + self.standby_race.get_racer_count()

    def columnCount(self, parent=QtCore.QModelIndex()):
        if self.active_race.get_racer_count() > 0 or self.standby_race.get_racer_count() > 0:
            return Racer.DATA_SIZE
        else:
            return 0
