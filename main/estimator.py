import numpy as np
from typing import List, Tuple

class Estimator:
    """
    論文の核心であるRL推定アルゴリズムを実装するクラス
    """
    def calc_direct_RL_estimate(self,
                                chi_hat_ij_i_k:np.ndarray,
                                noisy_v:np.ndarray,
                                noisy_d:float,
                                noisy_d_dot:float,
                                T:float,
                                gamma:float
                                ) -> np.ndarray:
        """
        論文の式(1)に基づき、直接相対自己位置推定（Direct RL Estimation）を計算
        Args:
            chi_hat_ij_i_k (np.ndarray): 現在の相対位置の推定値ベクトル (x̂_k)
            noisy_v (np.ndarray): ノイズを含む相対速度ベクトル (v_k + ε_k)
            noisy_d (float): ノイズを含む距離スカラー (d_k + ε_d)
            noisy_d_dot (float): ノイズを含む距離変化率スカラー (ḋ_k + ε_ḋ)
            T (float): サンプリング周期
            gamma (float): ゲインパラメータ (γ)
        Returns:
            np.ndarray: 次の時刻の相対自己位置の直接推定値 (x̂_{k+1})
        """
        # 式(1)を構成要素に分解
        # 第1項：現在の推定値
        current_RL_term = chi_hat_ij_i_k
        #print(f"現在の推定値：{current_estimate}")
        # 第2項：速度に基づく予測項
        predicton_term = T * noisy_v
        #print(f"予測項：{predicton_term}")
        # 第3項：観測誤差に基づく補正項
        # 角括弧[]内のスカラー誤差を計算
        scalar_error = (noisy_d * noisy_d_dot) - (noisy_v.T @ chi_hat_ij_i_k)
        #print(f"スカラー誤差{scalar_error}")
        # スカラー誤差を用いてベクトル補正項を計算
        correction_term = gamma * T * noisy_v * scalar_error
        #print(f"補正項{correction_term}")
        # 全ての項を結合して次の推定値を算出
        chi_hat_ij_i_k_plus_1 = current_RL_term + predicton_term + correction_term
        #print(f"次の直接推定値{chi_hat_ij_i_k_plus_1}")
        return chi_hat_ij_i_k_plus_1
    
    def calc_fused_RL_estimate(self,
                               pi_ij_i_k: np.ndarray,
                               direct_estimate_x_hat: np.ndarray,
                               indirect_estimates: List[np.ndarray],
                               noisy_v: np.ndarray,
                               T: float,
                               kappa_D: float,
                               kappa_I: float
    ) -> np.ndarray:
        """
        論文の式(5)に基づき，融合相対自己位置推定（fused RL estimation）を計算
        Args:
            pi_ij_i_k (np.ndarray): 現在の融合推定値 (π_k)
            direct_estimate_x_hat (np.ndarray): 自機で計算した直接推定式 (x̂_k)
            indirect_estimates (List[np.ndarray]): 隣接機から得られる間接推定値のリスト [x̂_{r,k}]
            noisy_v (np.ndarray): ノイズを含む相対速度ベクトル (v_k + ε_k)
            T (float): サンプリング周期
            kappa_D (float): 直接推定の重み (κ^D)
            kappa_I (float): 間接推定の重み (κ^I)
        Returns:
            np.ndarray: 次の時刻の相対自己位置の融合推定値 (π_{k+1})
        """
        # 式(5)を構成要素に分解
        # 第1項：現在の推定値
        current_fused_RL_term = pi_ij_i_k
        #print(f"現在の推定値: {current_fused_RL_term}")
        # 第2項：速度に基づく予測項
        prediction_term = T * noisy_v
        #print(f"予測項: {prediction_term}")
        # 第3項：直接推定による補正
        # κ^D * [x̂_k - π_k]
        direct_correction_term = kappa_D * (direct_estimate_x_hat - pi_ij_i_k)
        #print(f"直接推定による補正: {direct_correction_term}")
        # 第4項: 間接推定による補正 (総和)
        # Σ κ^I * [x̂_{r,k} - π_k]
        indirect_correction_sum_term = np.zeros(2) # 2次元ベクトルとして初期化
        #print(f"間接推定による補正項の初期値: {indirect_correction_sum_term}")
        if indirect_estimates:
            for x_hat_ij_r_k in indirect_estimates:
                indirect_correction_sum_term += kappa_I * (x_hat_ij_r_k - pi_ij_i_k)
        #print(f"間接推定による補正項: {indirect_correction_sum_term}")
        # 全ての項を結合して次の融合推定値を算出
        pi_ij_i_k_plus_1 = current_fused_RL_term + prediction_term + direct_correction_term + indirect_correction_sum_term
        #print(f"次の融合推定値: {pi_ij_i_k_plus_1}")
        return pi_ij_i_k_plus_1
    
    def calc_estimation_kappa(self, uav_neigbors: List[int], target_uav_id: int) -> Tuple[float, float]:
        """
        論文記載の式に基づき重みkappaを計算
        Args:
            uav_neigbors(List[int]): uav_i(自機)の隣接機UAV idグループ
            target_uav_id(int): 推定対象のUAV id
        Returns:
            Tuple [float, float]: [直接推定の重み, 融合推定の重み]
        """
        cardinality_Ni = len(uav_neigbors) # |Ni|
        #print(f"Ni: {cardinality_Ni}")
        alpha_ij = 1 if target_uav_id in uav_neigbors else 0 # αij
        #print(f"alpha_ij: {alpha_ij}")
        denominator = cardinality_Ni + 1 + alpha_ij # 重み項の分母

        kappa_D = alpha_ij / denominator # κ^D
        kappa_I = 1.0 / denominator # κ^I
        #print(f"kappa_D: {kappa_D}")
        #print(f"kappa_I: {kappa_I}")
        return kappa_D, kappa_I
