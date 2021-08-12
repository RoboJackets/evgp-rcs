# Race Control System User Manual

## Purpose:
The Race Control System provides a platform to logically control multiple autonomous vehicles in a race competition format. The RCS allows Admins to remotely start a race and notify vehicles to stop when the race is finished. The RCS follows the EVGP Autonomous Safety and Control
System Proposal provided in this repository.

### Warning:
The Race Control System (RCS) software is provided without warranty.
The RCS is not and should not be used as an emergency safety system.
Administration and Teams are expected to have safety measures in place and maintain safe control of vehicles.

## First Use:
This project uses [Poetry](https://python-poetry.org/) for dependency management. Install Poetry before continuing.
To run the Race Control System:
1. ``poetry install``
1. update the racers_list.yaml with team names and IP addresses
2. ``poetry run python evgp_rcs/gui.py``


## General Usage:

### Info
The RCS TCP server starts on your local machine IP at port 12017. Racers will use that IP and port to connect to as a TCP connection.  
Commands are sent with as a string of characters ``$RACE_STATE;`` and expect that format in responses. See the EVGP Autonomous Safety and Control System Proposal for more details on how the RCS communicates and any other information this user manual does not provide.

### Start Up GUI
1. Connect to a LAN
1. Assign RCS host computer a static IP
1. Assign Teams a static IP on the LAN
1. Fill in racers_list.yaml with Team names and IP
	- List racers as ``STATIC_IP: TEAM_NAME``
	- Example: ``123.123.123: RoboJackets``
1. Start the RCS gui with ``poetry run python evgp_rcs/gui.py``

### To Start a Race:
1. Select Teams from the Other Teams table and press Move to Active Race.
1. Wait for Teams to connect. Connected Teams appear white.
1. Once connected, follow the **Race Sequence**


### Race Sequence:
1. Press Grid Active Race
2. Wait for all Teams to report GRID_ACTIVE
	- GRID_ACTIVE Teams will show green, otherwise red
3. Press Start Race
	- If Teams do not report GREEN_GREEN back, they will show red
4. Press Red Flag Race
	- It is encouraged for you to Red Flag individual Teams at a safe location as they Finish the Race
5. Press E-Stop Race
6. Wait for all Teams to report E-Stopped
7. Press Finish Race
8. Remove Teams from Active Race


### Definitions:
- **Team**: Vehicle that can connect to the RCS and defined with a name and IP in the racers_list.yaml
- **Race**: A set of Teams competing and a State in the Race Sequence.
- **State/Race State**: IN_GARAGE, GRID_ACTIVE, GREEN_GREEN, RED_FLAG, RED_RED
- **Active Race**: The Race controlled by the RCS which is occuring or will occur next
- **Race Sequence**: Order in which a Race takes place from start to finish. See Race Sequence section.

### GUI Layout
##### Team Tables
- Left panel is an Active Race Table containing Teams in the Active Race.
- Right panel is an Other Teams Table containing all possible Teams not in Active Race.
- Each Team is a row in the table
- Only one Team can be selected at a time.

##### Table Highlights:
 - **Blue**: Team selected
 - **Gray**: Team not connected
 - **White**: Team connected
 - **Green**: Team is in RCS signaled state
 - **Red**: Team is not in RCS signaled state

##### Control Button Panels:
###### Admin controls:
- Move to Active Race: Move selected Team to the Active Race
- Remove from Active Race: Remove selected Team to the Active Race back to Other Teams Table

###### Team controls: Send signals to selected Team
- In Garage Team: sends ``IN_GARAGE``
- Red Flag Team: sends ``RED_FLAG``
- E-Stop Team: sends ``RED_RED``

###### Race controls: Send signals to all Teams in the Active Race
- Start Race: sends ``GREEN_GREEN``
- Red Flag Race: sends ``RED_FLAG``
- E-Stop Race: sends ``RED_RED``
- Finish Race: sends ``IN_GARAGE``
