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
                if (r < len(self.active_race)):
                    #active racers
                    return self.active_race[index.row()].index(c)
                else:
                    return self.standby_race[index.row()].index(c)
        def rowCount(self, parent=QtCore.QModelIndex()):
            return len(self.active_race) + len(self.standby_race)
        def columnCount(self, parent=QtCore.QModelIndex()):
            if self.active_race:
                return len(self.active_race[0])
            elif self.self.standby_race:
                return len(self.self.standby_race[0])
            else:
                return 0
