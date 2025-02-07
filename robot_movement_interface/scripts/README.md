# RMI API

## Concept

The `rmi_api.py` is a PoC to enable a python interface to the primitive function blocks like. Content is copied from the primitive function blocks of drag&bot and drastically simplified:
- Move Cartesian
- Set IO
- Get IO

## Usage

Use the drag&bot Robot Simulator for the following example.

Create a python script in this directory:
```shell
touch example.py
```

You can now use the following functions:
```python
import rmi_api

# close.gripper
rmi_api.grab()

# open gripper
rmi_api.open()

# move robot to euler goal x, y, z in mm
rmi_api.move_to(100, 200, 50)

# get io status of pin 1
status_io_1 = rmi_api.get_io(1)

# set io of pin 2 to high (true)
rmi_api.set_io(2, True)
```

Execute in your shell with:
```shell
python3 example.py
```

## The LLM connection

### 1.1. Setup: Code

1. Clone the GitHub [repository](https://github.com/dragandbot/dnb_robot_movement_interface) into the `dnb_catkin_ws/` folder.
2. Start/restart the dnb_runtime container and make sure it is building.
3. Open: `dnb-console`
4. Navigate to the directory `roscd robot_movement_interface/scripts/`
5. Create a virtual environment: `python3 -m venv venv`
6. Activate: `. venv/bin/activate`
7. Install the requirements: `pip3 install -r requirement.txt`
8. Paste an OpenAI API Key into the env file: `.env`

### 1.2. Setup: drag&bot

1. Open Component Manager
2. Load Component 'Robot Simulator' with robot 'Simbot'. (Default drag&bot setup)
3. Open Scenario Designer in drag&bot
4. Load the default scenario: *dnb_assets/scene.scn*
5. Load the gripper from: *dnb_assets/tcp_gripper.json*
6. Select the gripper as active in drag&bot

### 2. Usage

Now you are able to generate a drag&bot program with the ChatGPT API and a prompt.

1. Open: `dnb-console`
2. Go to dir: `cd dnb_catkin_ws/src/dnb_robot_movement_interface/robot_movement_interface/scripts`
3. Activate: `. venv/bin/activate`
4. Generate a python program execute: `python3 llm_script.py`
5. Execute this with drag&bot paste the Code into the file: `paste_python_script.py`
6. Execute with: `python3 paste_python_script.py`