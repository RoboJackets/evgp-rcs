import socket
import yaml
import logging
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.Qt import QSortFilterProxyModel
from race import Racer, RaceState
from rcsstatemanager import RCSStateManager


class RCSModel(QtCore.QAbstractTableModel):

    team_state_change_signal = pyqtSignal(str, RaceState) #ip of responder as string, response as RaceState
    race_state_change_signal = pyqtSignal(RaceState, bool) # new state as RaceState, isReady as bool

    def __init__(self, teams_list_file_path):
        super(RCSModel, self).__init__()
        self.active_race = []
        self.standby_race = []
        self.race_state_manager = RCSStateManager(self.active_race, self.race_state_change_signal)
        self.dataChanged.connect(self.race_state_manager.racer_data_updated)
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
            current_state = RaceState(self.data(self.index(r, Racer.STATE)))
            last_response = RaceState(self.data(self.index(r, Racer.LAST_RESPONSE)))
            if current_state == RaceState.GRID_ACTIVE and current_state == last_response:
                return QtGui.QColor(129, 199, 132) # GREEN when ready for race start
            elif current_state != RaceState.IN_GARAGE and current_state != last_response:
                return QtGui.QColor(255, 100, 100) # RED if requested state not reported back
            else:
                return QtGui.QColor(255, 255, 255) # white if normal
        else:
            return QtGui.QColor(190, 190, 190) # GRAY if not connected


    def team_state_change(self, tableIdx, state):
        ip = None
        if (tableIdx < len(self.active_race)):
            #active racer
            self.active_race[tableIdx].state = state
            ip = self.active_race[tableIdx].ip

        else:
            self.standby_race[tableIdx-len(self.active_race)].state = state
            ip = self.standby_race[tableIdx-len(self.active_race)].ip


        modelTopLeftIndex = self.index(tableIdx, 0)
        modelBottomRightIndex = self.index(tableIdx, Racer.DATA_SIZE-1)
        self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)

        self.team_state_change_signal.emit(ip, state)

    def race_state_change(self, state):
        new_state = self.race_state_manager.update_race_state(state)
        for r in self.active_race:
            r.state = new_state
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
            self.layoutAboutToBeChanged.emit()

            self.move_racer(index - len(self.active_race), self.standby_race, self.active_race)

            modelTopLeftIndex = self.index(0,0)
            modelBottomRightIndex = self.index(self.rowCount() - 1, Racer.DATA_SIZE)
            self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)

            self.layoutChanged.emit()
            return True
        else:
            logging.warning("Racer already in the active race.")
            return False

    #returns true or false if anything actually moved
    def move_to_standby_race(self, index):
        if index < len(self.active_race):
            self.layoutAboutToBeChanged.emit()

            self.move_racer(index - len(self.active_race), self.active_race, self.standby_race)

            modelTopLeftIndex = self.index(0,0)
            modelBottomRightIndex = self.index(self.rowCount() - 1, Racer.DATA_SIZE)
            self.dataChanged.emit(modelTopLeftIndex, modelBottomRightIndex)

            self.layoutChanged.emit()
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
                logging.error(f"Lost connection to active racer team {self.teams_list[ip]}")
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
