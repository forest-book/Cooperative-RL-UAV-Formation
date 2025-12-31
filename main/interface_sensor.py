from abc import ABC, abstractmethod
import numpy as np

class ISensor(ABC):
    """センサ情報取得メソッドのインタフェース"""
    @abstractmethod
    def get_velocity_info(self, uav_i, uav_j, delta_bar, *, add_vel_noise=False) -> np.ndarray:
        pass

    @abstractmethod
    def get_distance_info(self, uav_i, uav_j, dist_bound, *, add_dist_noise=False) -> float:
        pass

    @abstractmethod
    def get_distance_rate_info(self, uav_i, uav_j, dist_bound, *, add_dist_rate_noise=False) -> float:
        pass