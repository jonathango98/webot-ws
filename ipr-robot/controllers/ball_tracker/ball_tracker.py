from ipr_robot import IPRRobot
from robot_controls import RobotControls

if __name__ == "__main__":
    print("Starting Ball Tracker Controller...\n")
    robot = IPRRobot()
    robot_controls = RobotControls(robot)
    robot_controls.main()