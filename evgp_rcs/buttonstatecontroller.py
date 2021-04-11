import logging
from race import RaceState, Racer

# Watches buttons to manage what can and can't be pressed based on race state
# Handles notifications on text labels

class ButtonStateController():

    def __init__(self,
            grid_active_race_button,
            start_race_button,
            red_flag_race_button,
            eStop_race_button,
            finish_race_button,
            in_garage_team_button,
            move_racer_active_button,
            move_racer_standby_button,
            race_state_label,
            race_info_label
            ):

        self.grid_active_race_button = grid_active_race_button
        self.start_race_button = start_race_button
        self.red_flag_race_button = red_flag_race_button
        self.eStop_race_button = eStop_race_button
        self.finish_race_button = finish_race_button
        self.in_garage_team_button = in_garage_team_button
        self.move_racer_active_button = move_racer_active_button
        self.move_racer_standby_button = move_racer_standby_button
        self.race_state_label = race_state_label
        self.race_info_label = race_info_label

    # slot(RaceState, bool)
    def race_state_updated(self, state, isReady):

        if RaceState.IN_GARAGE == state:
            self.in_garage_state()
        elif RaceState.GRID_ACTIVE == state and isReady:
            self.grid_active_ready_state()
        elif RaceState.GRID_ACTIVE == state:
            self.grid_active_state()
        elif RaceState.GREEN_GREEN == state:
            self.green_green_state()
        elif RaceState.RED_FLAG == state:
            self.red_flag_state()
        elif RaceState.RED_RED == state and isReady:
            self.red_red_ready_state()
        elif RaceState.RED_RED == state:
            self.red_red_state()
        else:
            pass


    def enable_all_buttons(self):
        self.grid_active_race_button.setEnabled(True)
        self.start_race_button.setEnabled(True)
        self.red_flag_race_button.setEnabled(True)
        self.eStop_race_button.setEnabled(True)
        self.finish_race_button.setEnabled(True)
        self.in_garage_team_button.setEnabled(True)
        self.move_racer_active_button.setEnabled(True)
        self.move_racer_standby_button.setEnabled(True)

    def in_garage_state(self):
        self.race_state_label.setText("Race State: IN_GARAGE")
        self.race_info_label.setText("No Race running.")

        self.grid_active_race_button.setEnabled(True)
        self.start_race_button.setEnabled(False)
        self.red_flag_race_button.setEnabled(True)
        self.eStop_race_button.setEnabled(True)
        self.finish_race_button.setEnabled(True)
        self.in_garage_team_button.setEnabled(True)
        self.move_racer_active_button.setEnabled(True)
        self.move_racer_standby_button.setEnabled(True)

    def grid_active_state(self):
        self.race_state_label.setText("Race State: GRID_ACTIVE")
        self.race_info_label.setText("Waiting for all Racers to ready up.")
        #can't change racer while race going
        self.grid_active_race_button.setEnabled(False)
        self.start_race_button.setEnabled(False)
        self.red_flag_race_button.setEnabled(True)
        self.eStop_race_button.setEnabled(True)
        self.finish_race_button.setEnabled(False)
        self.in_garage_team_button.setEnabled(False)
        self.move_racer_active_button.setEnabled(False)
        self.move_racer_standby_button.setEnabled(False)

        #TODO: tooltips
        self.move_racer_active_button.setToolTip("Cannot move racer while Race is Active")

    def grid_active_ready_state(self):
        self.race_state_label.setText("Race State: GRID_ACTIVE")
        self.race_info_label.setText("All Racers are ready to start race.")

        self.grid_active_race_button.setEnabled(False)
        self.start_race_button.setEnabled(True)
        self.red_flag_race_button.setEnabled(True)
        self.eStop_race_button.setEnabled(True)
        self.finish_race_button.setEnabled(False)
        self.in_garage_team_button.setEnabled(False)
        self.move_racer_active_button.setEnabled(False)
        self.move_racer_standby_button.setEnabled(False)

    def green_green_state(self):
        self.race_state_label.setText("Race State: GREEN_GREEN")
        self.race_info_label.setText("Race in progress.")

        self.grid_active_race_button.setEnabled(False)
        self.start_race_button.setEnabled(False)
        self.red_flag_race_button.setEnabled(True)
        self.eStop_race_button.setEnabled(True)
        self.finish_race_button.setEnabled(False)
        self.in_garage_team_button.setEnabled(False)
        self.move_racer_active_button.setEnabled(False)
        self.move_racer_standby_button.setEnabled(False)

    def red_flag_state(self):
        self.race_state_label.setText("Race State: RED_FLAG")
        self.race_info_label.setText("Racers are stopping.")

        self.grid_active_race_button.setEnabled(False)
        self.start_race_button.setEnabled(False)
        self.red_flag_race_button.setEnabled(False)
        self.eStop_race_button.setEnabled(True)
        self.finish_race_button.setEnabled(False)
        self.in_garage_team_button.setEnabled(False)
        self.move_racer_active_button.setEnabled(False)
        self.move_racer_standby_button.setEnabled(False)

    def red_red_state(self):
        self.race_state_label.setText("Race State: RED_RED")
        self.race_info_label.setText("Race over. Waiting for racers to stop.")

        self.grid_active_race_button.setEnabled(False)
        self.start_race_button.setEnabled(False)
        self.red_flag_race_button.setEnabled(False)
        self.eStop_race_button.setEnabled(True)
        self.finish_race_button.setEnabled(False)
        self.in_garage_team_button.setEnabled(False)
        self.move_racer_active_button.setEnabled(False)
        self.move_racer_standby_button.setEnabled(False)

    def red_red_ready_state(self):
        self.race_state_label.setText("Race State: RED_RED")
        self.race_info_label.setText("Race over and all Racers reporting stopped. You may IN_GARAGE the race.")

        self.grid_active_race_button.setEnabled(False)
        self.start_race_button.setEnabled(False)
        self.red_flag_race_button.setEnabled(False)
        self.eStop_race_button.setEnabled(True)
        self.finish_race_button.setEnabled(True)
        self.in_garage_team_button.setEnabled(False)
        self.move_racer_active_button.setEnabled(False)
        self.move_racer_standby_button.setEnabled(False)
