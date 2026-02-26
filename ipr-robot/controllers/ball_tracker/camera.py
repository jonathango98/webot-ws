from controller import Robot
import cv2
import numpy as np
from typing import Tuple, Optional

class Camera:
    def __init__(self, robot: Robot, cam_name: str = "camera", screen_name: str = "display"):
        self.robot = robot
        self.cam_name = cam_name
        self.camera = self.robot.robot.getDevice(self.cam_name)
        self.camera.enable(self.robot.timestep)

        self.screen_name = screen_name
        self.display = self.robot.robot.getDevice(screen_name)

    def _get_image(self):
        image = self.camera.getImage()
        image = np.frombuffer(image, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4)).copy()

        return image
    
    def _get_object_center(self, image, color_range: Tuple = ((50, 120, 150), (70, 255, 255))) -> Optional[Tuple[int, int]]:
        bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, np.array(color_range[0]), np.array(color_range[1]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                return (cX, cY)
        return None
        
    def _draw_crosshair(self, image, size=10):
        center = self._get_object_center(image)
        if center:
            cX, cY = center
            cv2.line(image, (cX - size, cY), (cX + size, cY), (0, 0, 255, 255), 2)
            cv2.line(image, (cX, cY - size), (cX, cY + size), (0, 0, 255, 255), 2)
        return image
    
    def push_to_display(self, image):
        height, width = image.shape[:2]
        image_ref = self.display.imageNew(image.tobytes(), self.display.BGRA, width, height)
        self.display.imagePaste(image_ref, 0, 0, False)
        self.display.imageDelete(image_ref)

    def push_crosshair_display(self):
        image = self._get_image()
        crosshair_image = self._draw_crosshair(image)
        self.push_to_display(crosshair_image)