import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.Qt import QSortFilterProxyModel
from PyQt5.QtWidgets import QGridLayout, QGroupBox
from rcsmodel import RCSModel

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.table = QtWidgets.QTableView()

        self.model = RCSModel()
        #TODO: load based on server connections
        self.model.active_race.addRacer("team1", None, "123.123.123.123")
        self.model.active_race.addRacer("team2", None, "456.456.456.456")
        self.model.standby_race.addRacer("team3", None, "789.789.789.789")
        ##
        self.table.setModel(self.model)

        layout = QGridLayout() #TODO: better layout
        layout.setColumnStretch(0, 10)
        layout.setColumnStretch(1, 10)


        layout.addWidget(self.table, 0, 0)


        self.proxyModel = QSortFilterProxyModel()
        self.proxyModel.setDynamicSortFilter(True)
        self.proxyModel.setSourceModel(self.model)
        self.tableFiltered = QtWidgets.QTableView()
        self.tableFiltered.setModel(self.proxyModel)
        # self.proxyModel.setFilterKeyColumn(1)
        # self.proxyModel.setFilterFixedString("3")
        layout.addWidget(self.tableFiltered, 0, 1)




        self.horizontalGroupBox = QGroupBox("Grid")
        self.horizontalGroupBox.setLayout(layout)
        self.setCentralWidget(self.horizontalGroupBox)


app=QtWidgets.QApplication(sys.argv)
window=MainWindow()
window.show()
app.exec_()
