from interface_sensor import ISensor
import numpy as np
from quadcopter import UAV

class MockSensor(ISensor):
    """pythonのみでのフォーメーション制御検証用センサクラス"""
    def get_velocity_info(self, uav_i: UAV, uav_j: UAV, delta_bar, *, add_vel_noise=False) -> np.ndarray:
        true_v_ij = uav_j.true_velocity - uav_i.true_velocity
        # 速度ノイズ: ガウス分布 N(0, σ²)
        # 元の一様分布の全幅δ̄を±3σ（6σ）に対応させる → σ = δ̄/6
        sigma_v = delta_bar / 6.0
        vel_noise = np.random.normal(0, sigma_v, size=2) if add_vel_noise else np.zeros(2)
        return true_v_ij + vel_noise
    
    def get_distance_info(self, uav_i: UAV, uav_j: UAV, dist_bound, *, add_dist_noise=False) -> float:
        # 真の相対位置
        true_x_ij = uav_j.true_position - uav_i.true_position
        true_d_ij = np.linalg.norm(true_x_ij) # UWBモジュールでの測距を模している
        # 距離ノイズ: ガウス分布 N(0, σ²)
        # 元の一様分布の全幅boundを±3σ（6σ）に対応させる → σ = bound/6
        sigma_d = dist_bound / 6.0
        dist_noise = np.random.normal(0, sigma_d) if add_dist_noise else 0.0
        return true_d_ij + dist_noise
    
    def get_distance_rate_info(self, uav_i: UAV, uav_j: UAV, dist_bound, *, add_dist_rate_noise=False) -> float:
        true_x_ij = uav_j.true_position - uav_i.true_position
        true_v_ij = uav_j.true_velocity - uav_i.true_velocity
        true_d_ij = np.linalg.norm(true_x_ij)
        eps = 1e-9 # ゼロ除算防止
        true_d_dot_ij = (true_x_ij @ true_v_ij) / (true_d_ij + eps)
        # 距離変化率ノイズ: ガウス分布 N(0, σ²)
        # 距離ノイズと同じ標準偏差σ_dを使用
        sigma_d = dist_bound / 6.0
        dist_rate_noise = np.random.normal(0, sigma_d) if add_dist_rate_noise else 0.0
        return true_d_dot_ij + dist_rate_noise
