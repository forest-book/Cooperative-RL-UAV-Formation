import numpy as np
from typing import List, Union

class ControlInput:
    """
    UAVへの制御入力を担うクラス
    """
    def calc_RL_based_control_input(
            self,
            vel_i_k: np.ndarray,
            rel_v_ij_i_k: List[np.ndarray],
            rel_distances: List[float],
            desired_distances: Union[float, List[float]],
            pi_ij_i_k: List[np.ndarray],
            T: float,
            gamma1: float,
            gamma2: float,
            gamma2_min: float,
            gamma2_max: float,
            steepness: float = 0.05,
            error_center: float = 100.0,
            is_adaptive: bool = False
        ) -> np.ndarray:
        """
        式(9)に基づくフォーメーション制御入力の計算
        Args:
            vel_i_k (np.ndarray): 自機の現在の速度ベクトル v_{i,k}
            rel_v_ij_i_k (List[np.ndarray]): 隣接機との現在の相対速度ベクトルのリスト
            rel_distances (List[float]): 隣接機との現在の距離のリスト
            desired_distances (Union[float, List[float]]): 隣接UAVへの望ましい相対距離（スカラまたはリスト）
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


        # 第2項: 相対速度合意項
        # gamma_1 * T * Σ v_{i,k}^{ij} （隣接機jに対する相対速度v_{ij}の合計）
        rel_velocity_sum = np.zeros_like(vel_i_k, dtype=float)
        for v_ij in rel_v_ij_i_k:
            rel_velocity_sum += v_ij
        velocity_consensus_term = gamma1 * T * rel_velocity_sum
        #print(f"速度合意項: {velocity_consensus_term}")

        # 第3項: フォーメーション制御項
        # gamma_2 * T * Σ (d_{k}^{ij}^2 - d_{ij}^{*2}) * pi_{i,k}^{ij}
        rl_correction_sum = np.zeros_like(vel_i_k, dtype=float)
        for j in range(num_neighbors):
            d_ij = rel_distances[j]          # 現在の距離 d
            d_star = desired_dists_list[j]   # 目標距離 d* (リストから取得)
            pi_ij = pi_ij_i_k[j]             # 推定相対位置ベクトル pi

            # 距離誤差スカラ: (d^2 - d*^2)
            dist_error_scalar = (d_ij ** 2) - (d_star ** 2)
            # ベクトルへの重み付け加算
            rl_correction_sum += dist_error_scalar * pi_ij
        # 全隣接機の最大誤差で1つのgamma2を決定
        max_error = max(abs((d**2) - (d_star**2)) for d, d_star in zip(rel_distances, desired_dists_list))
        gamma2_adaptive = self.calc_adaptive_gamma2_sigmoid(
            dist_error_sq=max_error,
            gamma2_min=gamma2_min,
            gamma2_max=gamma2_max,
            steepness=steepness,
            error_center=error_center)
        if is_adaptive:
            gamma2 = gamma2_adaptive
        formation_control_term = gamma2 * T * rl_correction_sum
        #print(f"フォーメーション制御項: {formation_control_term}")
        vel_i_k_plus_1 = vel_i_k + velocity_consensus_term + formation_control_term
        #print(f"次ステップの制御入力(速度): {vel_i_k_plus_1}")
        return vel_i_k_plus_1
    
    def calc_adaptive_gamma2_sigmoid(
        self,
        dist_error_sq: float,
        gamma2_min: float,
        gamma2_max: float,
        steepness: float = 0.05,
        error_center: float = 100.0
    ) -> float:
        """
        シグモイド関数で距離誤差をgamma2に写像

        誤差が大きい → gamma2_min（抑制）
        誤差が小さい → gamma2_max（加速） 
        """
        abs_error = abs(dist_error_sq)

        # シグモイド関数: 誤差が小さいほど1に近づく
        sigmoid = 1.0 / (1.0 + np.exp(-steepness * (error_center - abs_error)))

        # [gamma2_min, gamma2_max] に線形写像
        gamma2 = gamma2_min + (gamma2_max - gamma2_min) * sigmoid

        return gamma2
