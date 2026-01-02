import numpy as np
from typing import List, Dict
from collections import defaultdict

class UAV:
    """
    各UAVの状態と機能を管理するクラス
    論文 V-A-1節「Configuration」に基づき、UAVのダイナミクスを定義
    """
    def __init__(self, uav_id: int, initial_position: np.ndarray, neighbors: List[int]):
        self.id: int = uav_id
        self.neighbors: List[int] = neighbors
        self.true_position: np.ndarray = np.array(initial_position, dtype=float)
        self.true_velocity: np.ndarray = np.array([0, 0], dtype=float)

        # 制御入力（次ステップの速度）
        self.control_input: np.ndarray = np.array([0, 0], dtype=float)

        # 推定値を保持する辞書 {target_id: estimate_vector}
        self.direct_estimates: Dict[str, List[np.ndarray]] = defaultdict(list)
        self.fused_estimates: Dict[str, List[np.ndarray]] = defaultdict(list)

    def update_state(self, dt: float):
        """UAVの真の位置と速度を更新する"""
        # 制御入力の速度式
        # 速度は [m/s] 単位として解釈し、dt を掛けて位置を更新
        self.true_velocity = self.control_input

        # 位置の更新: v [m/s] × dt [s] = 変位 [m]
        self.true_position += self.true_velocity * dt
        #print(f"uav_{self.id}の次ステップの位置")
        #print(self.true_position)
