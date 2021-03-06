import socket
import yaml
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import Qt
from race import Racer, RaceState


class RCSModel(QtCore.QAbstractTableModel):

    def __init__(self):
        super(RCSModel, self).__init__()
        self.active_race = []
        self.standby_race = []
        #TODO: disconnected or not connected racers from teams list should show
        self.teams_list = {}
        self.load_team_list()

    def load_team_list(self):
        #TODO: load a map from yaml file of team name to IP
        #self.teams_list = yaml.load(file("../racers_list.yaml"))
        self.teams_list = {
            "123.123.123.123": "team1",
            "456.456.456.456": "team2",
            "789.789.789.789": "team3",
            "127.0.0.1": "team4"
            }
        self.beginInsertRows(QtCore.QModelIndex(), self.rowCount(), self.rowCount() + len(self.teams_list))
        for ip, team in self.teams_list.items():
            r = Racer(team, ip)
            self.standby_race.append(r)
        self.endInsertRows()

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if role == Qt.DisplayRole:
            # data is tabled as rows active followed by standy
            # rows are filled with racer data (see race.py definition)
            r = index.row()
            c = index.column()
            if (r < len(self.active_race)):
                #active racers
                return self.active_race[r].index(c)
            else:
                return self.standby_race[r - len(self.active_race)].index(c)

        if role == Qt.BackgroundRole:
            if index.row() > len(self.active_race) - 1:
                return QtGui.QColor(190, 190, 190)

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self.active_race) + len(self.standby_race)

    def columnCount(self, parent=QtCore.QModelIndex()):
        if self.active_race or self.standby_race:
            return Racer.DATA_SIZE
        else:
            return 0

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if (role == Qt.DisplayRole):
            if (orientation == Qt.Horizontal):
                return QtCore.QVariant(Racer.HEADER_LABELS[section])
            else:
                return QtCore.QVariant(None)


    def race_state_change(self, state):
        for r in self.active_race:
            r.state = state
        modelTopLeftIndex = self.index(0,Racer.STATE)
        modelBottomRightIndex = self.index(len(self.active_race) - 1, Racer.STATE)
        self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)

    def move_racer(self, index, race1, race2):
        r = race1.pop(index)
        race2.append(r)

    def move_to_active_race(self, index):
        if index > len(self.active_race) - 1:
            self.move_racer(index - len(self.active_race), self.standby_race, self.active_race)
            #modelTopLeftIndex = self.index(self.active_race.get_racer_count() - 1,0)
            #modelBottomRightIndex = self.index(self.active_race.get_racer_count() + self.standby_race.get_racer_count() - 1, Racer.DATA_SIZE)

            modelTopLeftIndex = self.index(0,0)
            modelBottomRightIndex = self.index(self.rowCount() - 1, Racer.DATA_SIZE)
            self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)
        else:
            print("Racer already in the active race")

    def move_to_standby_race(self, index):
        if index < len(self.active_race):
            self.move_racer(index - len(self.active_race), self.standby_race, self.active_race)
            #modelTopLeftIndex = self.index(self.active_race.get_racer_count() - 1,0)
            #modelBottomRightIndex = self.index(self.active_race.get_racer_count() + self.standby_race.get_racer_count() - 1, Racer.DATA_SIZE)

            modelTopLeftIndex = self.index(0,0)
            modelBottomRightIndex = self.index(self.rowCount() - 1, Racer.DATA_SIZE)
            self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)
        else:
            print("Racer already in the active race")

    @QtCore.pyqtSlot(str)
    def new_connection_handler(self, ip):
        team = self.teams_list[ip]
        for i in range(len(self.standby_race)):
            r = self.standby_race[i]
            if r.ip == ip:
                r.is_connected = True
                modelTopLeftIndex = self.index(len(self.active_race)+i,Racer.IS_CONNECTED)
                self.dataChanged.emit(modelTopLeftIndex, modelTopLeftIndex)
                return

#TODO: everything below this
    @QtCore.pyqtSlot(str)
    def lost_connection_handler(self, ip):
        for i in range(len(self.active_race)):
            if self.active_race[i].ip == ip:
                self.active_race[i].is_connected = False
                self.move_to_standby_race(i)
                modelTopLeftIndex = self.index(i, Racer.IS_CONNECTED)
                self.dataChanged.emit(modelTopLeftIndex, modelTopLeftIndex)
                modelTopLeftIndex = self.index(len(self.standby_race), Racer.IS_CONNECTED)
                self.dataChanged.emit(modelTopLeftIndex, modelTopLeftIndex)
                print(f"Lost connection to active racer team {self.teams_list[ip]}") #TODO: report an error if in active race!
                return
        for i in range(len(self.standby_race)):
            if self.standby_race[i].ip == ip:
                self.standby_race[i].is_connected == False
                modelTopLeftIndex = self.index(len(self.active_race) + i, Racer.IS_CONNECTED)
                self.dataChanged.emit(modelTopLeftIndex, modelTopLeftIndex)
                return


    @QtCore.pyqtSlot(str, RaceState)
    def new_response_handler(self, ip, response):
        for i in range(len(self.active_race)): #TODO: ditch the whole Race class design
            if self.active_race[i].ip == ip:
                self.active_race[i].last_response = response
                return
        for i in range(len(self.standby_race)):
            if self.standby_race[i].ip == ip:
                self.standby_race[i].last_response = response
                return
