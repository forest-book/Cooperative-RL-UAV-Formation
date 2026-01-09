import numpy as np
from typing import Dict, Tuple

class MeasurementFilter:
    """測定値に対する指数移動平均フィルタ"""

    def __init__(self, alpha: float = 0.2):
        """
        Args:
            alpha (float): フィルタの平滑化係数 (0 < alpha < 1)
                           小さいほど平滑化が強くなる（ノイズ除去効果↑、応答速度↓）
        """
        self.alpha: float = alpha
        # 各UAVペアの前回フィルタ値を保持: {(i,j): (filtered_v, filtered_d, filtered_d_dot)}
        self.prev_filtered_values: Dict[Tuple[int, int], Tuple[np.ndarray, float, float]] = {}

    def apply(self, key: Tuple[int, int],
              measured_v: np.ndarray,
              measured_d: float,
              measured_d_dot: float) -> Tuple[np.ndarray, float, float]:
        """
        指定されたUAVペアに対して指数移動平均(EMA)フィルタを適用
        filtered_k = α * raw_k + (1-α) * filtered_{k-1}

        Args:
            key (Tuple[int, int]): UAVペアの識別子 (i, j)
            measured_v (np.ndarray): 測定された相対速度ベクトル
            measured_d (float): 測定された距離スカラー
            measured_d_dot (float): 測定された距離変化率スカラー

        Returns:
            Tuple[np.ndarray, float, float]: フィルタ後の相対速度、距離、距離変化率
        """
        if key not in self.prev_filtered_values:
            # 初回は測定値をそのまま使用
            self.prev_filtered_values[key] = (measured_v.copy(), measured_d, measured_d_dot)
            return measured_v, measured_d, measured_d_dot

        prev_v, prev_d, prev_d_dot = self.prev_filtered_values[key]

        # EMAフィルタ適用
        filtered_v = self.alpha * measured_v + (1 - self.alpha) * prev_v
        filtered_d = self.alpha * measured_d + (1 - self.alpha) * prev_d
        filtered_d_dot = self.alpha * measured_d_dot + (1 - self.alpha) * prev_d_dot

        # フィルタ後の値を保存
        self.prev_filtered_values[key] = (filtered_v.copy(), filtered_d, filtered_d_dot)

        return filtered_v, filtered_d, filtered_d_dot
    
    def reset(self):
        """フィルタの状態をリセット"""
        self.prev_filtered_values.clear()