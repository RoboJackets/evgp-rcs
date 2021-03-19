import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QThread, QItemSelection
from PyQt5.Qt import QSortFilterProxyModel
from PyQt5.QtWidgets import QGridLayout, QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton
from rcsmodel import RCSModel, RCSSortFilterProxyModel
from race import RaceState
from tcpserver import TCPServer

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


        start_server_button = QPushButton("Start Server")
        start_server_button.clicked.connect(self.start_server)
        verticalBoxLayout.addWidget(start_server_button)
        stop_server_button = QPushButton("Stop Server")
        stop_server_button.clicked.connect(self.stop_server)
        verticalBoxLayout.addWidget(stop_server_button)


        self.create_race_state_button_layout(layout, 1, 0)
        self.create_team_state_button_layout(layout, 1, 1)

        self.setCentralWidget(self.horizontalGroupBox)

    def create_race_state_button_layout(self, base_layout, row, col):
        hBox = QHBoxLayout()
        start_race_button = QPushButton("START RACE")
        start_race_button.clicked.connect(lambda: self.race_state_change_callback(RaceState.GREEN_GREEN))
        hBox.addWidget(start_race_button)
        gridActiveRaceButton = QPushButton("GRID ACTIVE RACE")
        gridActiveRaceButton.clicked.connect(lambda: self.race_state_change_callback(RaceState.GRID_ACTIVE))
        hBox.addWidget(gridActiveRaceButton)
        eStopRaceButton = QPushButton("E-STOP RACE")
        eStopRaceButton.clicked.connect(lambda: self.race_state_change_callback(RaceState.RED_RED))
        hBox.addWidget(eStopRaceButton)

        base_layout.addLayout(hBox, row, col)

    def create_team_state_button_layout(self, base_layout, row, col):
        hBox = QHBoxLayout()
        eStopTeamButton = QPushButton("E-STOP TEAM")
        eStopTeamButton.clicked.connect(lambda: self.team_state_change_callback(RaceState.RED_RED))
        hBox.addWidget(eStopTeamButton)
        slowTeamButton = QPushButton("SLOW TO STOP TEAM")
        slowTeamButton.clicked.connect(lambda: self.team_state_change_callback(RaceState.RED_FLAG))
        hBox.addWidget(slowTeamButton)
        inGarageTeamButton = QPushButton("IN GARAGE TEAM")
        inGarageTeamButton.clicked.connect(lambda: self.team_state_change_callback(RaceState.IN_GARAGE))
        hBox.addWidget(inGarageTeamButton)

        base_layout.addLayout(hBox, row, col)

    def team_state_change_callback(self, state):
        if self.selectedIndex is not None::
            self.model.team_state_change(self.selectedIndex, state)

    def race_state_change_callback(self, state):
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
        #TODO: handle this doesn't work to start, stop, and start again
        #TODO: handle address in use (server just shut down)
        if not self.is_server_started:
            self.is_server_started = True
            port = 8080
            server_backlog = 10
            self.server = TCPServer(port, server_backlog)
            self.server.new_connection.connect(self.model.new_connection_handler)
            self.server.lost_connection.connect(self.model.lost_connection_handler)
            self.server.new_response.connect(self.model.new_response_handler)

            self.model.team_state_change_signal.connect(self.server.on_race_state_change)

            self.server_thread = QThread()
            self.server.moveToThread(self.server_thread)
            self.server_thread.started.connect(self.server.run_server)
            self.server_thread.start()

    def stop_server(self):
        if self.is_server_started:
            self.server.stop() #TODO: make this a signal so it works. Also mutex?
            self.is_server_started = False

    @QtCore.pyqtSlot(QItemSelection, QItemSelection)
    def standby_race_table_selection_handler(self, filterTableSelection, filterTableDeselected):
        if filterTableSelection.indexes():
            self.activeRaceTable.selectionModel().clearSelection()
            self.selectedIndex = self.standbyRaceProxyModel.mapToSource(filterTableSelection.indexes()[0]).row()

    @QtCore.pyqtSlot(QItemSelection, QItemSelection)
    def active_race_table_selection_handler(self, tableSelection, tableDeselected):
        if tableSelection.indexes():
            self.standbyRaceTable.selectionModel().clearSelection()
            #self.selection = self.proxyModel.mapToSource(tableSelection.indexes()) #TODO: main table should be proxy
            self.selectedIndex = tableSelection.indexes()[0].row()

    def clearAllSelections(self):
        self.activeRaceTable.selectionModel().clearSelection()
        self.standbyRaceTable.selectionModel().clearSelection()
        self.selectedIndex = None


app=QtWidgets.QApplication(sys.argv)
window=MainWindow()
window.show()
app.exec_()
