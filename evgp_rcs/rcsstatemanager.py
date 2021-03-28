from race import RaceState, Racer
from enum import Enum

# Watches buttons to manage what can and can't be pressed based on state

#TODO: better integrate with RCSModel to avoid info duplication by moving signalled information there and just changing buttons here.

class RCSStateManager():

    def __init__(self,
            racers_list,
            grid_active_race_button,
            start_race_button,
            red_flag_race_button,
            eStop_race_button,
            in_garage_team_button,
            move_racer_active_button,
            move_racer_standby_button
            ):

        self.state = RaceState.IN_GARAGE
        self.racers_list = racers_list
        self.grid_active_race_button = grid_active_race_button
        self.start_race_button = start_race_button
        self.red_flag_race_button = red_flag_race_button
        self.eStop_race_button = eStop_race_button
        self.in_garage_team_button = in_garage_team_button
        self.move_racer_active_button = move_racer_active_button
        self.move_racer_standby_button = move_racer_standby_button

        #TODO: make this work
        # self.grid_active_race_button.clicked.connect(self.gridActiveState)
        # self.start_race_button.clicked.connect(self.raceStartState)
        # self.red_flag_race_button.clicked.connect(self.redFlagState)
        # self.eStop_race_button.clicked.connect(self.eStopState)

        #TODO: this is a hack that may cause memory leaks. Figure out why above doesn't work
        self.grid_active_race_button.clicked.connect(lambda: self.gridActiveState())
        self.start_race_button.clicked.connect(lambda: self.raceStartState())
        self.red_flag_race_button.clicked.connect(lambda: self.redFlagState())
        self.eStop_race_button.clicked.connect(lambda: self.eStopState())

    #trigger when GRID ACTIVE race button pressed
    def gridActiveState(self):
        self.state = RaceState.GRID_ACTIVE

        #can't change racer while race going
        self.move_racer_active_button.setEnabled(False)
        self.move_racer_standby_button.setEnabled(False)
        #disable other buttons
        self.grid_active_race_button.setEnabled(False)
        self.start_race_button.setEnabled(False)
        self.in_garage_team_button.setEnabled(False)

        self.move_racer_active_button.setToolTip("Cannot move racer while Race is Active")

        #self.gridActiveStateReady() #TODO: need this to check once


    #trigger when update of racer state from tcpserver
    def gridActiveStateReady(self):
        if self.state == RaceState.GRID_ACTIVE:
            if self.racersAgree(self.racers_list) == RaceState.GRID_ACTIVE:
                print("Racers ready to start")
                self.grid_active_race_button.setEnabled(False)
                self.start_race_button.setEnabled(True)
            else:
                print("Racers no longer ready")
                self.gridActiveState() #if racer become unready

    #trigger when START RACE button pressed
    def raceStartState(self):
        self.state = RaceState.GREEN_GREEN
        self.start_race_button.setEnabled(False)

        print("Race is GREEN_GREEN")

    #trigger when RED FLAG race button pressed
    def redFlagState(self):
        self.state = RaceState.RED_FLAG

    #trigger when E-STOP/RED_RED/Finish race button pressed
    #TODO: maybe seperate finish race button
    def eStopState(self):
        self.state = RaceState.RED_RED
        self.red_flag_race_button.setEnabled(False)
        #self.eStopStateReady() #TODO: need to check once when btn pressed

    #trigger when update of racer state from tcpserver
    def eStopStateReady(self):
        if self.state == RaceState.RED_RED:
            if self.racersAgree(self.racers_list) == RaceState.RED_RED:
                #race is over, allow moving into race
                self.move_racer_active_button.setEnabled(True)
                self.move_racer_standby_button.setEnabled(True)
                self.move_racer_active_button.setToolTip("")
                #re-enable buttons
                self.start_race_button.setEnabled(True)
                self.grid_active_race_button.setEnabled(True)
                self.in_garage_team_button.setEnabled(True)
            else:
                print("Racers no longer E-Stopped!")
                self.eStopState() #if racer become unready


    #returns RaceState that all racers are or None if no agreement
    def racersAgree(self, racers_list):
        state = None
        for r in racers_list:
            if state is None:
                state = r.last_response
            elif state != r.last_response:
                return None
        return state
