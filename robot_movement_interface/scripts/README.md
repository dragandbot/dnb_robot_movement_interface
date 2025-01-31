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