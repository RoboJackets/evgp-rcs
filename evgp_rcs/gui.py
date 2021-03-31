import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QThread, QItemSelection
from PyQt5.Qt import QSortFilterProxyModel
from PyQt5.QtWidgets import QGridLayout, QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QMessageBox
from rcsmodel import RCSModel, RCSSortFilterProxyModel
from race import RaceState
from tcpserver import TCPServer

from rcsstatemanager import RCSStateManager

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.is_server_started = False

        self.model = RCSModel()

        layout = QGridLayout() #TODO: better layout
        layout.setColumnStretch(0, 10)
        layout.setColumnStretch(1, 10)


        #FILTERED ACTIVE RACE TABLE
        self.activeRaceProxyModel = RCSSortFilterProxyModel(True)
        self.activeRaceProxyModel.setDynamicSortFilter(True)
        self.activeRaceProxyModel.setSourceModel(self.model)

        self.activeRaceTable = QtWidgets.QTableView()
        self.activeRaceTable.setModel(self.activeRaceProxyModel)
        self.activeRaceTable.setSizeAdjustPolicy(
            QtWidgets.QAbstractScrollArea.AdjustToContents)
        self.activeRaceTable.setSelectionBehavior(QtWidgets.QTableView.SelectRows)
        self.activeRaceTable.setSelectionMode(QtWidgets.QTableView.SingleSelection)
        layout.addWidget(self.activeRaceTable, 0, 0)


        self.standbyRaceProxyModel = RCSSortFilterProxyModel(False)
        self.standbyRaceProxyModel.setDynamicSortFilter(True)
        self.standbyRaceProxyModel.setSourceModel(self.model)

        self.standbyRaceTable = QtWidgets.QTableView()
        self.standbyRaceTable.setModel(self.standbyRaceProxyModel)
        self.standbyRaceTable.setSizeAdjustPolicy(
            QtWidgets.QAbstractScrollArea.AdjustToContents)
        self.standbyRaceTable.setSelectionBehavior(QtWidgets.QTableView.SelectRows)
        self.standbyRaceTable.setSelectionMode(QtWidgets.QTableView.SingleSelection)
        layout.addWidget(self.standbyRaceTable, 0, 1)


        self.race_state_label = QLabel("Race State: IN_GARAGE")
        self.teams_state_label = QLabel("") #TODO: add to rcsstatemanager
        layout.addWidget(self.race_state_label, 2, 0)

        # self.standbyRaceTable.setSortingEnabled(True) #TODO: this breaks everything

        self.selectedIndex = None
        self.standbyRaceTable.selectionModel().selectionChanged.connect(self.standby_race_table_selection_handler)
        self.activeRaceTable.selectionModel().selectionChanged.connect(self.active_race_table_selection_handler)



        self.horizontalGroupBox = QGroupBox("Grid")
        self.horizontalGroupBox.setLayout(layout)

        verticalBoxLayout = QVBoxLayout()
        move_to_active_race_button = QPushButton("Move to Active Race")
        move_to_active_race_button.clicked.connect(self.move_to_active_race)
        verticalBoxLayout.addWidget(move_to_active_race_button)
        remove_from_active_race_button = QPushButton("Remove from Active Race")
        remove_from_active_race_button.clicked.connect(self.remove_from_active_race)
        verticalBoxLayout.addWidget(remove_from_active_race_button)


        layout.addLayout(verticalBoxLayout, 0, 2)

        # Race State Control Buttons
        race_state_btns = self.create_race_state_buttons()
        race_state_layout = QHBoxLayout()
        for btn in race_state_btns:
            race_state_layout.addWidget(btn)
        layout.addLayout(race_state_layout, 1, 0)

        # Team State Control Buttons
        team_state_btns = self.create_team_state_buttons()
        team_state_layout = QHBoxLayout()
        for btn in team_state_btns:
            team_state_layout.addWidget(btn)
        layout.addLayout(team_state_layout, 1, 1)

        rcsstate = RCSStateManager(
            self.model.active_race,
            race_state_btns[0],
            race_state_btns[1],
            race_state_btns[2],
            race_state_btns[3],
            team_state_btns[2],
            move_to_active_race_button,
            remove_from_active_race_button
        )
        self.model.dataChanged.connect(rcsstate.gridActiveStateReady)
        self.model.dataChanged.connect(rcsstate.eStopStateReady)


        # wait for start of server
        self.server_wait_label = QLabel("Waiting for TCP Server to start. Please hold on.")
        self.setCentralWidget(self.server_wait_label)
        self.start_server()

    # Make sure we stop the server on window close
    def closeEvent(self, event):
        self.stop_server()
        event.accept()

    #TODO: add in garage team btn or remove from RCS documentation
    def create_race_state_buttons(self):
        grid_active_race_button = QPushButton("GRID ACTIVE RACE")
        grid_active_race_button.clicked.connect(lambda: self.race_state_change_callback(RaceState.GRID_ACTIVE))
        start_race_button = QPushButton("START RACE")
        start_race_button.setEnabled(False)
        start_race_button.clicked.connect(lambda: self.race_state_change_callback(RaceState.GREEN_GREEN))
        red_flag_race_button = QPushButton("RED FLAG RACE")
        red_flag_race_button.clicked.connect(lambda: self.race_state_change_callback(RaceState.RED_FLAG))
        e_stop_race_button = QPushButton("E-STOP RACE")
        e_stop_race_button.clicked.connect(lambda: self.race_state_change_callback(RaceState.RED_RED))

        return [grid_active_race_button, start_race_button, red_flag_race_button, e_stop_race_button]


    def create_team_state_buttons(self):
        e_stop_team_button = QPushButton("E-STOP TEAM")
        e_stop_team_button.clicked.connect(lambda: self.team_state_change_callback(RaceState.RED_RED))
        red_flag_team_button = QPushButton("SLOW TO STOP TEAM")
        red_flag_team_button.clicked.connect(lambda: self.team_state_change_callback(RaceState.RED_FLAG))
        in_garage_team_button = QPushButton("IN GARAGE TEAM")
        in_garage_team_button.clicked.connect(lambda: self.team_state_change_callback(RaceState.IN_GARAGE))

        return [e_stop_team_button, red_flag_team_button, in_garage_team_button]

    def team_state_change_callback(self, state):
        if self.selectedIndex is not None:
            self.model.team_state_change(self.selectedIndex, state)

    def race_state_change_callback(self, state):
        self.race_state_label.setText(f"Race State: {state}")
        self.model.race_state_change(state)

    def move_to_active_race(self):
        if self.selectedIndex is not None:
            changed = self.model.move_to_active_race(self.selectedIndex)
            if changed:
                self.clearAllSelections()

    def remove_from_active_race(self):
        if self.selectedIndex is not None:
            changed = self.model.move_to_standby_race(self.selectedIndex)
            if changed:
                self.clearAllSelections()

    def start_server(self):
        if not self.is_server_started:
            self.is_server_started = True
            port = 8080
            server_backlog = 10
            ip_list = self.model.teams_list.keys()
            self.server = TCPServer(port, server_backlog, whitelist=ip_list)
            self.server.new_connection.connect(self.model.new_connection_handler)
            self.server.lost_connection.connect(self.model.lost_connection_handler)
            self.server.new_response.connect(self.model.new_response_handler)

            self.server.server_ready.connect(self.server_ready_handler)

            self.model.team_state_change_signal.connect(self.server.on_race_state_change)

            self.server_thread = QThread()
            self.server.moveToThread(self.server_thread)
            self.server_thread.started.connect(self.server.run_server)
            self.server_thread.start()

    def stop_server(self):
        if self.is_server_started:
            self.server.stop()
            self.is_server_started = False
            self.server_thread.quit()

    @QtCore.pyqtSlot(bool)
    def server_ready_handler(self, isReady):
        if isReady:
            self.setCentralWidget(self.horizontalGroupBox)
        if not isReady:
            self.server_wait_label.setText("Server Error: Please restart program.")
            QMessageBox.question(self, 'Server Error',
                                "Server failed to start.\nPress OK to quit program, fix your network issues, then restart this program.",
                                QMessageBox.Ok)
            self.close()

    @QtCore.pyqtSlot(QItemSelection, QItemSelection)
    def standby_race_table_selection_handler(self, filterTableSelection, filterTableDeselected):
        if filterTableSelection.indexes():
            self.activeRaceTable.selectionModel().clearSelection()
            self.selectedIndex = self.standbyRaceProxyModel.mapToSource(filterTableSelection.indexes()[0]).row()

    @QtCore.pyqtSlot(QItemSelection, QItemSelection)
    def active_race_table_selection_handler(self, tableSelection, tableDeselected):
        if tableSelection.indexes():
            self.standbyRaceTable.selectionModel().clearSelection()
            self.selectedIndex = self.activeRaceProxyModel.mapToSource(tableSelection.indexes()[0]).row()

    def clearAllSelections(self):
        self.activeRaceTable.selectionModel().clearSelection()
        self.standbyRaceTable.selectionModel().clearSelection()
        self.selectedIndex = None


app=QtWidgets.QApplication(sys.argv)
window=MainWindow()
window.show()
app.exec_()
