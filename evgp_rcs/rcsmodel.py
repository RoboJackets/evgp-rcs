import socket
import yaml
import logging
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.Qt import QSortFilterProxyModel
from race import Racer, RaceState


class RCSModel(QtCore.QAbstractTableModel):

    team_state_change_signal = pyqtSignal(str, RaceState) #ip of responder as string, response as RaceState

    def __init__(self, teams_list_file_path):
        super(RCSModel, self).__init__()
        self.active_race = []
        self.standby_race = []
        #TODO: disconnected or not connected racers from teams list should show
        self.teams_list = {}
        self.load_team_list(teams_list_file_path)

    def load_team_list(self, filepath):
        self.teams_list={}
        try:
            with open(filepath) as f:
                self.teams_list = yaml.load(f, Loader=yaml.FullLoader)
        except:
            pass
        self.beginInsertRows(QtCore.QModelIndex(), self.rowCount(), self.rowCount() + len(self.teams_list))
        for ip, team in self.teams_list.items():
            r = Racer(team, ip)
            self.standby_race.append(r)
        self.endInsertRows()

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if role == Qt.DisplayRole:
            # data is tabled as rows active followed by standby
            # rows are filled with racer data (see race.py definition)
            r = index.row()
            c = index.column()
            if (r < len(self.active_race)):
                #active racers
                return self.active_race[r].index(c)
            else:
                return self.standby_race[r - len(self.active_race)].index(c)

        if role == Qt.BackgroundRole:
            return self.decideRowColor(index)

        # if role == Qt.ToolTipRole: #SHOWS FULL TEXT ON HOVER
        #     return self.data(index,role=Qt.DisplayRole)

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

    def decideRowColor(self, index):
        r = index.row()
        if self.data(self.index(r, Racer.IS_CONNECTED)):
            #Connected
            if self.data(self.index(r, Racer.STATE)) != self.data(self.index(r, Racer.LAST_RESPONSE)):
                return QtGui.QColor(255, 100, 100)
        else:
            return QtGui.QColor(190, 190, 190)


    def team_state_change(self, tableIdx, state):
        ip = None
        if (tableIdx < len(self.active_race)):
            #active racer
            self.active_race[tableIdx].state = state
            ip = self.active_race[tableIdx].ip

        else:
            #TODO: should standby racers be allowed to be in GO states?
            self.standby_race[tableIdx-len(self.active_race)].state = state
            ip = self.standby_race[tableIdx-len(self.active_race)].ip


        modelTopLeftIndex = self.index(tableIdx, 0)
        modelBottomRightIndex = self.index(tableIdx, Racer.DATA_SIZE-1)
        self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)

        self.team_state_change_signal.emit(ip, state)

    def race_state_change(self, state):
        for r in self.active_race:
            r.state = state
            self.team_state_change_signal.emit(r.ip, state)
        modelTopLeftIndex = self.index(0,0)
        modelBottomRightIndex = self.index(len(self.active_race) - 1, Racer.DATA_SIZE-1)
        self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)

    def move_racer(self, index, race1, race2):
        r = race1.pop(index)
        race2.append(r)

    #returns true or false if anything actually moved
    def move_to_active_race(self, index):
        if index > len(self.active_race) - 1:
            self.layoutAboutToBeChanged.emit() #TODO: see if this is right

            # if  not self.standby_race[index].is_connected: #TODO: ENABLE AFTER TESTING
            #     print("Racer not connected!")
            #     return
            self.move_racer(index - len(self.active_race), self.standby_race, self.active_race)
            #modelTopLeftIndex = self.index(self.active_race.get_racer_count() - 1,0)
            #modelBottomRightIndex = self.index(self.active_race.get_racer_count() + self.standby_race.get_racer_count() - 1, Racer.DATA_SIZE)

            modelTopLeftIndex = self.index(0,0)
            modelBottomRightIndex = self.index(self.rowCount() - 1, Racer.DATA_SIZE)
            self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)

            self.layoutChanged.emit() #TODO: see if this is right
            return True
        else:
            logging.warning("Racer already in the active race.")
            return False

    #returns true or false if anything actually moved
    def move_to_standby_race(self, index):
        if index < len(self.active_race):
            self.layoutAboutToBeChanged.emit() #TODO: see if this is right

            self.move_racer(index - len(self.active_race), self.active_race, self.standby_race)
            #modelTopLeftIndex = self.index(len(self.active_race.get_racer_count() - 1,0)
            #modelBottomRightIndex = self.index(self.active_race.get_racer_count() + self.standby_race.get_racer_count() - 1, Racer.DATA_SIZE)

            modelTopLeftIndex = self.index(0,0)
            modelBottomRightIndex = self.index(self.rowCount() - 1, Racer.DATA_SIZE)
            self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)

            self.layoutChanged.emit() #TODO: see if this is right
            return True
        else:
            logging.warning("Racer not in active race.")
            return False

    @QtCore.pyqtSlot(str)
    def new_connection_handler(self, ip):
        team = self.teams_list.get(ip)
        if team is None:
            return
        for i in range(len(self.active_race)):
            r = self.active_race[i]
            if r.ip == ip:
                r.is_connected = True
                modelTopLeftIndex = self.index(i, 0)
                modelBottomRightIndex = self.index(i, Racer.DATA_SIZE-1)
                self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)
                return

        for i in range(len(self.standby_race)):
            r = self.standby_race[i]
            if r.ip == ip:
                r.is_connected = True
                modelTopLeftIndex = self.index(len(self.active_race) + i, 0)
                modelBottomRightIndex = self.index(len(self.active_race) + i, Racer.DATA_SIZE-1)
                self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)
                return

    @QtCore.pyqtSlot(str)
    def lost_connection_handler(self, ip):
        for i in range(len(self.active_race)):
            if self.active_race[i].ip == ip:
                self.active_race[i].is_connected = False
                modelTopLeftIndex = self.index(i, 0)
                modelBottomRightIndex = self.index(i, Racer.DATA_SIZE-1)
                self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)
                logging.error(f"Lost connection to active racer team {self.teams_list[ip]}") #TODO: show UI error in table if in active race
                return
        for i in range(len(self.standby_race)):
            if self.standby_race[i].ip == ip:
                self.standby_race[i].is_connected = False
                modelTopLeftIndex = self.index(len(self.active_race) + i, 0)
                modelBottomRightIndex = self.index(len(self.active_race) + i, Racer.DATA_SIZE-1)
                self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)
                return


    @QtCore.pyqtSlot(str, RaceState)
    def new_response_handler(self, ip, response):
        #TODO: check speed of this with 15 racers. Could have faster lookup
        for i in range(len(self.active_race)):
            if self.active_race[i].ip == ip:
                self.active_race[i].last_response = response
                modelTopLeftIndex = self.index(i, 0)
                modelBottomRightIndex = self.index(i, Racer.DATA_SIZE-1)
                self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)
                return
        for i in range(len(self.standby_race)):
            if self.standby_race[i].ip == ip:
                self.standby_race[i].last_response = response
                modelTopLeftIndex = self.index(len(self.active_race) + i, 0)
                modelBottomRightIndex = self.index(len(self.active_race) + i, Racer.DATA_SIZE-1)
                self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)
                return



class RCSSortFilterProxyModel(QSortFilterProxyModel):

    def __init__(self, filterAcceptsActive, parent=None):
        super(QSortFilterProxyModel, self).__init__(parent=parent)

        self.filterAcceptsActive = filterAcceptsActive

    def filterAcceptsRow(self, sourceRow, sourceParent):
        if self.filterAcceptsActive:
            return sourceRow < len(self.sourceModel().active_race)
        else:
            return sourceRow >= len(self.sourceModel().active_race)
