# EVGP Race Control System (RCS)

Created for the [EVGrandPrix (EVGP) Autonomous Kart Racing Competition](https://evgrandprix.org/autonomous/).
Karts can connect to the RCS through a simple TCP socket.
The GUI allows management of a race by signaling the race start sequence, race ending, and E-Stops. It is up to teams to handle and respect race signals and act safely during manual and autonomous control.

See EVGP-RCS document for more documentation <TODO: Move that info here>



## Setup

This project uses [Poetry](https://python-poetry.org/) for dependency management. Install Poetry before continuing.
PyQt 5 is used for the GUI framework.

### To Use:
1. Connect to a LAN
1. Set your IP to a static
1. Give each team a static IP and have them set their IP
1. Clone repo, navigate to repo in terminal
1. ``poetry install``
1. Place a ``racer_list.yaml`` in repo top level folder
    - List racers as ``STATIC_IP: TEAM_NAME``
    - Example: ``123.123.123: RoboJackets``
1. ``poetry run python evgp_rcs/gui.py``
1. In the GUI, select racers and press move to active race
1. Run race following EVGP-RCS documentation
    - When ready, press GRID_ACTIVE, then START_RACE
    - When finished, press E-STOP RACE


``tcpclient.py`` provides an example TCP client that can be adopted for a Race vehicle

### Develop
1. Clone repo, navigate to repo in terminal
1. ``poetry install``
1. Write code
1. ``poetry run python evgp_rcs/gui.py``
1. To test: While running the GUI, ``poetry run python evgp_rcs/tcpclient.py``
1. tcplient.py has multiple modes to uncomment, default set to press number keys for sending different state messages.
