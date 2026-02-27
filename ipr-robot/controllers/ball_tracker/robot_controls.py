from ipr_robot import IPRRobot
from camera import Camera
from controller import Keyboard
import math

class RobotControls:
    def __init__(self, robot: IPRRobot):
        self.robot = robot
        self.camera = Camera(self.robot, cam_name="camera", screen_name="display")

        self.keyboard = Keyboard()
        self.keyboard.enable(robot.timestep)

        # 2. P-Gains (Tune these if it moves too aggressively or too slowly)
        self.Kp_r = 0.0001
        self.Kp_theta = 0.0001

        self.r_step, self.theta_step, self.z_step, self.roll_step = 0.005, 0.025, 0.005, 0.1
        self.prev_key = -1

    def auto_approach(self, z_target=0.025):
        """Image-Based Visual Servoing to center and lower the arm."""
        image = self.camera._get_image()
        center = self.camera._get_object_center(image)
        
        if not center:
            print("Target lost! Stopping auto-approach.")
            return

        obj_x, obj_y = center
        img_cx = self.camera.camera.getWidth() / 2
        img_cy = self.camera.camera.getHeight() / 5 * 3

        # 1. Calculate Pixel Error
        err_x = img_cx - obj_x
        err_y = img_cy - obj_y
        
        # 3. Apply corrections to Robot's Polar Coordinates
        # Image Y error maps to reaching forward/backward (r)
        self.robot.r += err_y * self.Kp_r 
        
        # Image X error maps to rotating left/right (theta)
        # Note: Depending on your camera's exact rotation, you may need to flip the sign to '-='
        self.robot.theta += err_x * self.Kp_theta 
        
        # 4. Auto-Descend
        if self.robot.z > z_target: # Target height just above the cube
            self.robot.z -= 0.002

        # 5. Apply safety limits
        self.robot.r = max(0.1, min(self.robot.r, 0.7))
        self.robot.z = max(0.01, min(self.robot.z, 0.8))

        # 6. Command the movement
        x = self.robot.r * math.cos(self.robot.theta)
        y = self.robot.r * math.sin(self.robot.theta)
        self.robot.move([x, y, self.robot.z])

    def step_key(self, key: int):
        # Spacebar Toggle
        if self.prev_key != key and key == ord(' '):
            self.robot.set_gripper()

        # Keyboard Controls
        if key == Keyboard.UP:    self.robot.r += self.r_step
        if key == Keyboard.DOWN:  self.robot.r -= self.r_step
        if key == Keyboard.LEFT:  self.robot.theta += self.theta_step
        if key == Keyboard.RIGHT: self.robot.theta -= self.theta_step
        if key == ord('W'):       self.robot.z += self.z_step
        if key == ord('S'):       self.robot.z -= self.z_step
        if key == ord('A'):       self.robot.roll += self.roll_step
        if key == ord('D'):       self.robot.roll -= self.roll_step
        if key == ord('P'):       self.auto_approach()

        # Safety Clipping
        self.robot.r = max(0.1, min(self.robot.r, 0.7))
        self.robot.z = max(0.01, min(self.robot.z, 0.8))
        self.robot.roll = max(-3.14, min(self.robot.roll, 3.14))

        self.prev_key = key

        target_pos = self.polar_to_cartesian(self.robot.r, self.robot.theta, self.robot.z)
        self.robot.move(target_pos, self.robot.roll)

        print(f"Target Pose -> r: {self.robot.r:.3f}, θ: {math.degrees(self.robot.theta):.1f}°, z: {self.robot.z:.3f}, roll: {math.degrees(self.robot.roll):.1f}°      ", end="\r", flush=True)    

    def cartesian_to_polar(self, position):
        x, y, z = position
        r = math.sqrt(x**2 + y**2)
        theta = math.atan2(y, x)
        return r, theta, z
    
    def polar_to_cartesian(self, r, theta, z):
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        return [x, y, z]

    def main(self):
        while self.robot.robot.step(self.robot.timestep) != -1:
            self.camera.push_crosshair_display()
            key = self.keyboard.getKey() 
            self.step_key(key)