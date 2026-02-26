import os
import math
import ikpy.chain
import numpy as np
from controller import Robot, Keyboard

class IPRRobot:
    def __init__(self):
        self.robot: Robot = Robot()
        self.timestep: int = int(self.robot.getBasicTimeStep())

        # Hardware Constants
        self.GRIP_OPEN_ANGLE = 1.22
        self.GRIP_CLOSED_ANGLE = 0.0
        self.KP = 15.0  # Proportional Gain - Adjust for snappiness

        # Pose Variables (The 'Ghost' Target)
        self.r, self.theta, self.z, self.roll = 0.0, 0.0, 0.3, 0.0
        self.r_step, self.theta_step, self.z_step, self.roll_step = 0.005, 0.025, 0.005, 0.1

        self.prev_key = -1
        self.urdf_path = os.path.join(os.getcwd(), "urdf/ipr.urdf")

        self.gripper_open = False

        self._initialize()

    def _initialize(self):
        motor_names = ['base', 'upperarm', 'forearm', 'wrist', 'rotational_wrist']
        self.motors = []
        self.max_velocities = []
        
        for name in motor_names:
            motor = self.robot.getDevice(name)
            # Enable Velocity Mode
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            
            # Hardware-aware speed limits
            limit = motor.getMaxVelocity()
            self.max_velocities.append(limit if limit > 0 else 1.0)
            
            sensor = motor.getPositionSensor()
            sensor.enable(self.timestep)
            self.motors.append(motor)
        
        self.left_gripper = self.robot.getDevice('gripper::left')
        self.right_gripper = self.robot.getDevice('gripper::right')
        
        grip_force_limit = 5.0 # Newtons/Nm - Tune this!
        
        self.left_gripper.setAvailableForce(grip_force_limit)
        self.right_gripper.setAvailableForce(grip_force_limit)

        # Setup IK Chain
        base_chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path)
        active_mask = [link.name in motor_names for link in base_chain.links]
        self.my_chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path, active_links_mask=active_mask)

    def step_key(self, key: int):
        # Spacebar Toggle
        if self.prev_key != key and key == ord(' '):
            self._set_gripper()

        # Keyboard Controls
        if key == Keyboard.UP:    self.r += self.r_step
        if key == Keyboard.DOWN:  self.r -= self.r_step
        if key == Keyboard.LEFT:  self.theta += self.theta_step
        if key == Keyboard.RIGHT: self.theta -= self.theta_step
        if key == ord('W'):       self.z += self.z_step
        if key == ord('S'):       self.z -= self.z_step
        if key == ord('A'):       self.roll += self.roll_step
        if key == ord('D'):       self.roll -= self.roll_step

        # Safety Clipping
        self.r = max(0.1, min(self.r, 0.7))
        self.z = max(0.01, min(self.z, 0.8))
        self.roll = max(-3.14, min(self.roll, 3.14))

        self.prev_key = key

        # 1. Coordinate Transform
        x = self.r * math.cos(self.theta)
        y = self.r * math.sin(self.theta)
        target_pos = [x, y, self.z]
        self.move(target_pos, self.roll)

        print(f"Target Pose -> r: {self.r:.3f}, θ: {math.degrees(self.theta):.1f}°, z: {self.z:.3f}, roll: {math.degrees(self.roll):.1f}°")

    def inverse_kinematics(self, target_pos) -> np.ndarray:
        current_q = [0.0] * len(self.my_chain.links)
        for i in range(len(self.motors)):
            val = self.motors[i].getPositionSensor().getValue()
            current_q[i+1] = val if not math.isnan(val) else 0.0
        return self.my_chain.inverse_kinematics(target_pos, initial_position=current_q)

    def move(self, target_pos, roll=None):
        if roll is None:
            roll = self.roll

        # 1. Get the ACTUAL current joint positions from sensors
        actual_q = [0.0] * len(self.my_chain.links)
        for i in range(len(self.motors)):
            val = self.motors[i].getPositionSensor().getValue()
            actual_q[i+1] = val if not math.isnan(val) else 0.0

        try:
            # 2. Solve IK using actual_q as the starting point
            target_q = self.my_chain.inverse_kinematics(target_pos, initial_position=actual_q)
            
            # 3. P-Control Loop
            for i in range(len(self.motors)):
                goal = target_q[i+1] if i < 4 else roll
                
                error = goal - actual_q[i+1]
                
                p_vel = error * self.KP
                limit = self.max_velocities[i]
                final_vel = max(-limit, min(p_vel, limit))
                
                if abs(error) < 0.002:
                    self.motors[i].setVelocity(0.0)
                else:
                    self.motors[i].setVelocity(final_vel)

        except ValueError:
            for m in self.motors: m.setVelocity(0.0)

    def _set_gripper(self):
        if self.gripper_open:
            self.left_gripper.setPosition(self.GRIP_CLOSED_ANGLE)
            self.right_gripper.setPosition(self.GRIP_CLOSED_ANGLE)
        else:
            self.left_gripper.setPosition(-self.GRIP_OPEN_ANGLE)
            self.right_gripper.setPosition(self.GRIP_OPEN_ANGLE)
        self.gripper_open = not self.gripper_open

    def get_end_effector_position(self):
        # result is (x, y, z) of the end-effector in world coordinates
        return self.my_chain.forward_kinematics([0.0] * len(self.my_chain.links))[:3, 3]
    
