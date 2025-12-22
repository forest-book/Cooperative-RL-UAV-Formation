from abc import ABC, abstractmethod
from interface_sensor import ISensor
import numpy as np
from quadcopter import UAV

class CoppeliaSensor(ISensor):
    
    def get_velocity_info(self, uav_i: UAV, uav_j: UAV, delta_bar, *, add_vel_noise=False) -> np.ndarray:
        return np.array([100.2, 100.5], dtype=float)