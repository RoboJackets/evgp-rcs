import socket
import yaml
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import Qt
from race import Race, Racer, RaceState


class RCSModel(QtCore.QAbstractTableModel):

    def __init__(self):
        super(RCSModel, self).__init__()
        self.active_race = Race()
        self.standby_race = Race()
        #TODO: disconnected or not connected racers from teams list should show
        self.teams_list = {}
        self.load_team_list()

    def load_team_list(self):
        #TODO: load a map from yaml file of team name to IP
        self.teams_list = {
            "123.123.123.123": "team1",
            "456.456.456.456": "team2",
            "789.789.789.789": "team3",
            "127.0.0.1": "team4"
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

        if role == Qt.BackgroundRole:
            if index.row() > self.active_race.get_racer_count() - 1:
                return QtGui.QColor(190, 190, 190)

    def rowCount(self, parent=QtCore.QModelIndex()):
        return self.active_race.get_racer_count() + self.standby_race.get_racer_count()

    def columnCount(self, parent=QtCore.QModelIndex()):
        if self.active_race.get_racer_count() > 0 or self.standby_race.get_racer_count() > 0:
            return Racer.DATA_SIZE
        else:
            return 0

    def race_state_change(self, state):
        self.active_race.race_state_change(state)
        modelTopLeftIndex = self.index(0,Racer.STATE)
        modelBottomRightIndex = self.index(self.active_race.get_racer_count() - 1, Racer.STATE)
        self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)

    def move_racer(self, index, race1, race2):
        r = race1.removeRacer(index)
        race2.addRacer(r)

    def move_to_active_race(self, index):
        if index > self.active_race.get_racer_count() - 1:
            self.move_racer(index - self.active_race.get_racer_count(), self.standby_race, self.active_race)
            #modelTopLeftIndex = self.index(self.active_race.get_racer_count() - 1,0)
            #modelBottomRightIndex = self.index(self.active_race.get_racer_count() + self.standby_race.get_racer_count() - 1, Racer.DATA_SIZE)

            modelTopLeftIndex = self.index(0,0)
            modelBottomRightIndex = self.index(3, Racer.DATA_SIZE)
            self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)
        else:
            print("Racer already in the active race")
    def move_to_standby_race(self, index):
        pass

    @QtCore.pyqtSlot(str)
    def new_connection_handler(self, ip):
        team = self.teams_list[ip]
        self.beginInsertRows(QtCore.QModelIndex(), self.rowCount(), self.rowCount())
        self.standby_race.createNewRacer(team, None, ip)
        self.endInsertRows()

    @QtCore.pyqtSlot(str)
    def lost_connection_handler(self, ip):
        for i in range(len(self.active_race.racers)): #TODO: ditch the whole Race class design
            if self.active_race.racers[i].ip == ip:
                self.beginRemoveRows(QtCore.QModelIndex(), i, i)
                self.active_race.removeRacer(i)
                self.endRemoveRows()
                print(f"Lost connection to active racer team {self.teams_list[ip]}") #TODO: report an error if in active race!
                return
        for i in range(len(self.standby_race.racers)): #TODO: ditch the whole Race class design
            if self.standby_race.racers[i].ip == ip:
                self.beginRemoveRows(QtCore.QModelIndex(), i + len(self.active_race.racers), i + len(self.active_race.racers))
                self.standby_race.removeRacer(i)
                self.endRemoveRows()
                return


    @QtCore.pyqtSlot(str, RaceState)
    def new_response_handler(self, ip, response):
        for i in range(len(self.active_race.racers)): #TODO: ditch the whole Race class design
            if self.active_race.racers[i].ip == ip:
                self.active_race.racers[i].last_response = response
                return
        for i in range(len(self.standby_race.racers)):
            if self.standby_race.racers[i].ip == ip:
                self.standby_race.racers[i].last_response = response
                return
