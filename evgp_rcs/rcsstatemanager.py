import logging
from race import RaceState, Racer

# Manages Race state

class RCSStateManager():

    def __init__(self,
            racers_list, #ref to RCSModel racers list
            state_signal #ref to RCSModel signal
            ):
        self.state = RaceState.IN_GARAGE
        self.is_ready = False
        self.racers_list = racers_list
        self.race_state_change_signal = state_signal

    def update_race_state(self, new_state):
        if self.can_transition(new_state):
            self.state = new_state
            logging.info(f"Race State is now {self.state}")
        self.is_ready = self.racers_ready(self.state)
        self.race_state_change_signal.emit(self.state, self.is_ready)
        return self.state

    #trigger when we get new data in
    def racer_data_updated(self):
        self.is_ready = self.racers_ready(self.state)
        self.race_state_change_signal.emit(self.state, self.is_ready)

    #returns True if state transition allowed
    def can_transition(self, next_state):
        if self.state == RaceState.RED_RED and self.is_ready:
            return next_state == RaceState.IN_GARAGE #only transition to in garage
        elif self.state == RaceState.RED_FLAG:
            return next_state == RaceState.RED_RED
        elif self.state == RaceState.GREEN_GREEN:
            return next_state == RaceState.RED_FLAG or next_state == RaceState.RED_RED
        elif self.state == RaceState.GRID_ACTIVE:
            if next_state == RaceState.GREEN_GREEN and self.is_ready:
                return True
            else:
                return next_state != RaceState.GREEN_GREEN
        elif self.state == RaceState.IN_GARAGE:
            return next_state != RaceState.GREEN_GREEN
        else:
            pass

    #returns if racers all report the state given
    def racers_ready(self, state):
        for r in self.racers_list:
            if r.is_connected and r.last_response != state:
                return False
        return True
