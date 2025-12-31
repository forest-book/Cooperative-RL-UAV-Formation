from abc import ABC, abstractmethod

class ISensor(ABC):
    
    @abstractmethod
    def get_velocity_info(self, uav_i, uav_j, delta_bar, *, add_vel_noise=False):
        pass

    @abstractmethod
    def get_distance_info(self, uav_i, uav_j, dist_bound, *, add_dist_noise=False) -> float:
        pass