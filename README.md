# EVGP Race Control System (RCS)

Created for the [EVGrandPrix (EVGP) Autonomous Kart Racing Competition](https://evgrandprix.org/autonomous/).
Karts can connect to the RCS through a simple TCP socket.
The GUI allows management of a race by signaling the race start sequence, race ending, and E-Stops. It is up to teams to handle and respect race signals and act safely during manual and autonomous control.

#### WARNING:
The Race Control System (RCS) software is provided without warranty.
The RCS is not and should not be used as an emergency safety system.
Administration and Teams are expected to have safety measures in place and maintain safe control of vehicles.

See EVGP Autonomous Safety and Control System Proposal for more documentation and the *RCS User Manual* for more information on the RCS GUI usage.



## Setup

This project uses [Poetry](https://python-poetry.org/) for dependency management. Install Poetry before continuing.
PyQt 5 is used for the GUI framework.

### A Brief How to Use:
1. Install poetry, clone repo, navigate to repo in terminal
1. Connect to a LAN
1. Set your computer IP to a static IP as the server
1. Give each team a static IP and have them set their IP
1. ``poetry install``
1. Place a ``racer_list.yaml`` in repo top level folder
    - List racers as ``STATIC_IP: TEAM_NAME``
    - Example: ``123.123.123: RoboJackets``
1. ``poetry run python evgp_rcs/gui.py``
1. In the GUI, select racers and press move to active race
1. Run race following the guide in the *RCS User Manual*


``tcpclient.py`` provides an example TCP client that can be adapted for a Race vehicle.
## Using tcpclient.py
1. Start RCS GUI
1. ``poetry run python tcpclient.py``
1. tcp client provides some examples ``poetry run python tcpclient.py --help``
    >\# TCP Client Examples ``--type`` options:\
    >\# well-behaved: Responds with state server requested\
    >\# interval: Iterates through RaceStates and sends the next one (please prove a --delay)\
    >\# single-message: Sends the message from the --message argument\
    >\# interactive: Use number keys 1-5 to select a RaceState to send


### How to Develop
1. Clone repo, navigate to repo in terminal
1. ``poetry install``
1. Write code
1. ``poetry run python evgp_rcs/gui.py``
1. To test: While running the GUI, ``poetry run python evgp_rcs/tcpclient.py``
 - tcplient.py has multiple configurations to assist in testing
