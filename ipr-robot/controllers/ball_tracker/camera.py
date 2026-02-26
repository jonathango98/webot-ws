from controller import Robot
import cv2
import numpy as np

class Camera:
    def __init__(self, robot: Robot, name: str):
        self.robot = robot
        self.name = name
        self.camera = self.robot.robot.getDevice(self.name)
        self.camera.enable(self.robot.timestep)

    def get_image(self):
        return self.camera.getImage()