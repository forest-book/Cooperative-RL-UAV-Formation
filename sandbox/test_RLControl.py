import numpy as np
from typing import List, Union

def calc_RL_based_control_input(
        vel_i_k: np.ndarray,
        rel_v_ij_i_k: List[np.ndarray],
        rel_distances: List[float],
        desired_distances: Union[float, List[float]],
        pi_ij_i_k: List[np.ndarray],
        T: float,
        gamma1: float,
        gamma2: float,
    ) -> np.ndarray:
    """
    式(9)に基づくフォーメーション制御入力の計算
    Args:
        vel_i_k (np.ndarray): 自機の現在の速度ベクトル v_{i,k}
        rel_v_ij_i_k (List[np.ndarray]): 隣接機との現在の相対速度ベクトルのリスト
        rel_distances (List[float]): 隣接機との現在の距離のリスト
        desired_distances (List[float]): 隣接UAVへの望ましい相対距離のリスト
        pi_ij_i_k (List[np.ndarray]): 隣接機への融合RL推定位置ベクトルのリスト
        T (float): サンプリング周期
        gamma1 (float): 速度減衰ゲイン
        gamma2 (float): フォーメーション制御ゲイン
    Returns:
        np.ndarray: 次のステップの制御入力(速度ベクトル v_{i,k+1})
    """
    # 入力データの整合性チェック (リスト長が一致しているか)
    num_neighbors = len(rel_v_ij_i_k)
    if not (len(rel_distances) == num_neighbors and len(pi_ij_i_k) == num_neighbors):
        raise ValueError("Input lists (velocities, distances, estimates) must have the same length.")
    
    # 目標距離がスカラ(float)で与えられた場合、リストに変換して扱う
    if isinstance(desired_distances, (float, int)):
        desired_dists_list = [float(desired_distances)] * num_neighbors
    else:
        desired_dists_list = desired_distances
        if len(desired_dists_list) != num_neighbors:
            raise ValueError("desired distance list must match the number of neighbors.")
        
    
    # 第2項: 速度合意項
    # gamma_1 * T * Σ v_{i,k}^{ij}
    rel_velocity_sum = np.zeros_like(rel_v_ij_i_k, dtype=float)
    for v_ij in rel_v_ij_i_k:
        rel_velocity_sum += v_ij
    velocity_consensus_term = gamma1 * T * rel_velocity_sum
    print(f"速度合意項: {velocity_consensus_term}")

    # 第3項: フォーメーション制御項
    # gamma_2 * T * Σ (d_{k}^{ij}^2 - d_{ij}^{*2}) * pi_{i,k}^{ij}
    rl_correction_sum = np.zeros_like(pi_ij_i_k, dtype=float)
    for i in range(num_neighbors):
        d_ij = rel_distances[i]          # 現在の距離 d
        d_star = desired_distances[i]    # 目標距離 d*
        pi_ij = pi_ij_i_k[i]             # 推定相対位置ベクトル pi

        # 距離誤差スカラ: (d^2 - d*^2)
        dist_error_scalar = (d_ij ** 2) - (d_star ** 2)
        # ベクトルへの重み付け加算
        rl_correction_sum += dist_error_scalar * pi_ij
    formation_control_term = gamma2 * T * rl_correction_sum
    print(f"フォーメーション制御項: {formation_control_term}")
    vel_i_k_plus_1 = vel_i_k + velocity_consensus_term + formation_control_term
    print(f"次ステップの制御入力(速度): {vel_i_k_plus_1}")
    return vel_i_k_plus_1 

# テストデータ
v_curr = np.array([0.5, 0.0])  # 自機の現在の速度

# 隣接機が2機いると仮定
# 1. 相対速度 v_ij (観測値)
rel_vels = [np.array([0.1, 0.1]), np.array([-0.2, 0.0])]
# 2. 現在の距離 d_ij (観測値)
dists = [14.0, 16.0]
# 3. 融合推定値 pi (推定値)
pis = [np.array([10.0, 10.0]), np.array([-10.0, 5.0])]

# パラメータ
target_d = 15.0  # 目標距離
dt = 0.05
g1 = 1.0
g2 = 0.014

# 計算実行
v_next = calc_RL_based_control_input(
    current_velocity=v_curr,
    relative_velocities=rel_vels,
    current_distances=dists,
    fused_estimates=pis,
    target_distances=target_d,
    T=dt,
    gamma_1=g1,
    gamma_2=g2
)

print(f"Current Velocity: {v_curr}")
print(f"Next Velocity:    {v_next}")