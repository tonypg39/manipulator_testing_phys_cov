
import numpy as np
from controller import ArmController
from pyquaternion import Quaternion


noise_sigmas = {
    "pose": 0.003,
    "controller": 0.055,
    "joint": 0.005
}


def add_noise_to_pose(pose, mu=0,sigma=noise_sigmas['pose']):
    pose.position.x += np.random.normal(mu,sigma)
    pose.position.y += np.random.normal(mu,sigma)
    return pose
    # pose.z += np.random.normal(mu,sigma)


def add_noise_to_joint_angles(joints, mu=0,sigma=0.005):
    for j in range(len(joints)):
        joints[j] += np.random.normal(mu,sigma)
    return joints

class NoiseController():
    """
    Perform the same operations as controller with some added noise
    """
    def __init__(self):
        self.controller = ArmController()
        # Noise parameters
        self.mu = 0
        self.sigma = noise_sigmas['controller']
    
    def move(self, dx=0, dy=0, dz=0, delta_quat=Quaternion(1, 0, 0, 0), blocking=True):
        # Inject noise in values 
        #...
        dx = dx + np.random.normal(self.mu, self.sigma) if dx is not None else dx
        # dy = dy + np.random.normal(self.mu, self.sigma) if dy is not None else dy
        # dz = dz + np.random.normal(self.mu, self.sigma) if dz is not None else dz
        self.controller.move(dx,dy,dz,delta_quat,blocking)
        
    def move_to(self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True):
        # Inject noise in values 
        #...
        x = x + np.random.normal(self.mu, self.sigma) if x is not None else x
        # y = y + np.random.normal(self.mu, self.sigma) if y is not None else y
        # z = z + np.random.normal(self.mu, self.sigma) if z is not None else z
        self.controller.move_to(x,y,z,target_quat,z_raise,blocking)
    
    def get_gripper_pose(self):
        return self.controller.get_gripper_pose()

