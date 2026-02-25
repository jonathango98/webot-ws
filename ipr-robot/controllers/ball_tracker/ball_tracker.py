from controller import Keyboard
from ipr_robot import IPRRobot
from camera import Camera

robot = IPRRobot()

keyboard = Keyboard()
keyboard.enable(robot.timestep)

camera = Camera(robot, "camera")

# --- Main Loop ---
print("--- IPR POLAR CONTROL READY ---")
print("ARROWS: Radius/Theta | W/S: Height | SPACE: Gripper")

while robot.robot.step(robot.timestep) != -1:
    camera.get_image()
    key = keyboard.getKey() 
    robot.step(key)