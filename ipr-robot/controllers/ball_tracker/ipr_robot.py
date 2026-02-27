import os
import math
import ikpy.chain
import numpy as np
from controller import Robot, Keyboard

import warnings
warnings.filterwarnings("ignore", category=UserWarning)  # Suppress IKPy warnings
class IPRRobot:
    def __init__(self):
        self.robot: Robot = Robot()
        self.timestep: int = int(self.robot.getBasicTimeStep())

        # Hardware Constants
        self.GRIP_OPEN_ANGLE = 1.22
        self.GRIP_CLOSED_ANGLE = 0.0
        self.KP = 15.0  # Proportional Gain - Adjust for snappiness

        self.urdf_path = os.path.join(os.getcwd(), "urdf/ipr.urdf")

        self.gripper_open = True
        self.grip_force_limit = 5.0 # Newtons/Nm - Tune this!

        self._initialize()

        # Pose Variables (The 'Ghost' Target)
        self.r, self.theta, self.z = self.get_initial_position()
        self.roll = 0.0

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
        
        self.left_gripper.setAvailableForce(self.grip_force_limit)
        self.right_gripper.setAvailableForce(self.grip_force_limit)
        
        # Setup IK Chain
        base_chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path)
        
        # Build a safe mask: Must be in motor_names AND must be a movable joint
        active_mask = []
        for i, link in enumerate(base_chain.links):
            is_requested_motor = link.name in motor_names
            is_actually_movable = base_chain.active_links_mask[i] # IKPy knows if it's 'fixed'
            
            active_mask.append(is_requested_motor and is_actually_movable)
            
        self.my_chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path, active_links_mask=active_mask)

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

    def set_gripper(self):
        if self.gripper_open:
            self.left_gripper.setPosition(self.GRIP_CLOSED_ANGLE)
            self.right_gripper.setPosition(self.GRIP_CLOSED_ANGLE)
        else:
            self.left_gripper.setPosition(-self.GRIP_OPEN_ANGLE)
            self.right_gripper.setPosition(self.GRIP_OPEN_ANGLE)
        self.gripper_open = not self.gripper_open

    def get_initial_position(self):
        return self.my_chain.forward_kinematics([0.0] * len(self.my_chain.links))[:3, 3]
    
