from openai import OpenAI
from dotenv import load_dotenv
# opens the local .env file with you OpenAI API key
load_dotenv()

from rich.console import Console
from rich.markdown import Markdown

client = OpenAI()
console = Console()

completion = client.chat.completions.create(
  model="gpt-4",
  messages=[
    {"role": "system", "content": """
      Let’s suppose you are a robotics engineer and are tasked to create a sequence of actions for an industrial robot system that describes the necessary steps to fulfill a task.
      I will now first describe the industrial robot system and then the task that must be fulfilled. You operate in a 3D Space. You work in a X,Y,Z coordinate system. X denotes width, Y denotes height, Z denotes depth. 0.0,0.0,0.0 is the default space origin.
      The unit for the 3D space is given in millimeters.

      The industrial robot system consists of the following components:
      - robot: a cartesian robot arm with six degrees of freedom and a robot flange where tools can be mounted
      - gripper: a finger gripper for prismatic parts. The finger gripper is permanently attached to the robot flange. To grip a part the gripper must be closed. To release a part the gripper must be opened.
      - camera: a camera in a fixed position that can detect if a part is ok or not ok if it is placed in the field of view of the camera
      - cubes: Six cubes which needs to be picked up and quality checked by a robot
      - Not_ok_bin: a bin for parts that are not ok
      - handover position: a fixture where parts need to be placed that must be handed over to the operator

      You will receive from the user a list of objects with its description, location and size.
      Each object in the list is part of the same 3D environment. The list of objects will be given by a enumeration like the following:
      - Object 1
      - Object 2
      - Object 3

      Each object in the enumeration will be represented in a CSV (comma-separated values) format. The csv format will look like the following:
      [object_name, object_description, x, y, z, sx, sy, sz]

      The meaning of entry in the CSV is the following:
      - object_name: the name of the object
      - object_description: a brief description of the object to give context
      - x: coordinate of the object on the X axis
      - y: coordinate of the object on the Y axis
      - z: coordinate of the object on the Z axis
      - sx: the size of the object in X axis
      - sy: the size of the object in Y axis
      - sz: the size of the object in Z axis

      The CSV list to the used industrial environment does look as follows:
      - Cube 1, A cube which needs to be picked up by a robot, 100, 200, 15, 30, 30, 30
      - Cube 2, A cube which needs to be picked up by a robot, 140, 200, 15, 30, 30, 30
      - Cube 3, A cube which needs to be picked up by a robot, 180, 200, 15, 30, 30, 30
      - Cube 4, A cube which needs to be picked up by a robot, 100, 250, 15, 30, 30, 30
      - Cube 5, A cube which needs to be picked up by a robot, 140, 250, 15, 30, 30, 30
      - Cube 6, A cube which needs to be picked up by a robot, 180, 250, 15, 30, 30, 30
      - Not_ok_bin, A cubical bin where production parts that are not ok are being placed, 500, 200, 100, 300, 300, 200
      - Handover position, The target location where the cubes should be placed, 500, -225, 0, 0, 0, 0
      - Camera, A camera in a fixed position that can detect if a part is ok or not ok if the part is placed in the field of view of the camera, -100, 200, 200, 50, 50, 50

      The initial state of the industrial robot system is as follows:
      - The robot is in the scene
      - It is undefined if the gripper is open or closed
      - the handover position is empty

      The task:
      - the industrial robot system must check every cube if it is ok or not
      - Cubes that are ok must be put back to its original position
      - Cubes that are not ok must be dropped in the Not_ok_bin
      - After all parts have been checked, the robot should stop above the 6 cubes and the gripper should be open
      """
    },
    {"role": "user", "content": """
      Please create a sequence of actions to fulfill the described task and create a table with a row for each each action and the following columns:
      - Action number: unique id
      - Action name: meaningful name for the action
      - Action description: what happens during this action
      - Situation After Action Completion: what is the situation after completion of this specific action
      - Robot movements: describe in detail the robot movements happening in this action. Can be more than one. If no robot movements take place, please write <NoMove>. Do not use real values here, just refer to the column of robot positions.
      - Robot positions:  list the robot positions that are relevant for the action. If robot positions are relevant for more than one action, please use the same id for the position in all actions.
      - Gripper actions: what happens with the griper in this action. If the gripper does nothing, please write <NoAction>.

      After you created a table with the sequence of actions. I want you to generate the actual python code to make the program run.
      In addition use the python module "rmi_api" to import the 4 functions defined above called move_to, grab, release and check_cube at the top of the python script.
      This is how the move_to function definition looks like `def move_to(x: float, y: float, z: float).
      The function check_cube will return a true value when cube was ok and a false value when not.
      Output the whole python script."""
    }
  ],
  temperature=0.0
)

console.print(Markdown(completion.choices[0].message.content))