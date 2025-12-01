import numpy as np
from typing import List, Dict
from collections import defaultdict
from enum import Enum, auto

class Scenario(Enum):
    CONTINUOUS = auto()
    SUDDEN_TURN = auto()

class UAV:
    """
    各UAVの状態と機能を管理するクラス
    論文 V-A-1節「Configuration」に基づき、UAVのダイナミクスを定義
    """
    def __init__(self, uav_id: int, initial_position: np.ndarray):
        self.id: int = uav_id
        self.true_position = np.array(initial_position, dtype=float)
        
        k = 0
        # 論文記載の速度式
        if self.id == 1:
            self.true_velocity = np.array([np.cos(k / 3), -5/3 * np.sin(k / 3)])
        elif self.id == 2:
            self.true_velocity = np.array([-2 * np.sin(k), 2 * np.cos(k)]) # 論文のv_2kのy成分はsin(k)だが、軌跡からcos(k)の誤植と判断
        elif self.id == 3:
            self.true_velocity = np.array([np.cos(k/5) - np.sin(k/5) * np.cos(k), np.sin(k/5) + np.cos(k/5) * np.cos(k)])
        elif self.id == 4:
            self.true_velocity = np.array([-3 * np.sin(k), 3 * np.cos(k)])
        elif self.id == 5:
            self.true_velocity = np.array([1/6, 0])
        elif self.id == 6:
            self.true_velocity = np.array([-10/3 * np.sin(k/3), 5/3 * np.cos(k/3)])
        else:
            raise ValueError("Invalid uav_id")

        # 推定値を保持する辞書 {target_id: estimate_vector}
        self.direct_estimates: Dict[str, List[np.ndarray]] = defaultdict(list)
        self.fused_estimates: Dict[str, List[np.ndarray]] = defaultdict(list)
        self.neighbors: List[int] = []

    def update_state(self, t: int, dt: float, event: Scenario = Scenario.CONTINUOUS):
        """UAVの真の位置と速度を更新する"""
        k = t * dt  # 速度式内部のkなので実時間に変換
        
        # 論文記載の速度式
        # 注: 添え字のkは離散時間ステップだが，速度式内部のkは実時間であるみたい
        # 速度は [m/s] 単位として解釈し、dt を掛けて位置を更新
        if self.id == 1:
            self.true_velocity = np.array([np.cos(k / 3), -5/3 * np.sin(k / 3)])
        elif self.id == 2:
            self.true_velocity = np.array([-2 * np.sin(k), 2 * np.cos(k)]) # 論文のv_2kのy成分はsin(k)だが、軌跡からcos(k)の誤植と判断
        elif self.id == 3:
            self.true_velocity = np.array([np.cos(k/5) - np.sin(k/5) * np.cos(k), np.sin(k/5) + np.cos(k/5) * np.cos(k)])
        elif self.id == 4:
            self.true_velocity = np.array([-3 * np.sin(k), 3 * np.cos(k)])
        elif self.id == 5:
            self.true_velocity = np.array([1/6, 0])
        elif self.id == 6:
            self.true_velocity = np.array([-10/3 * np.sin(k/3), 5/3 * np.cos(k/3)])

        # シナリオ2: UAV4の急な機動変更イベント
        if self.id == 4 and event == Scenario.SUDDEN_TURN and 100 <= k < 101:
            self.true_velocity += np.array([5.0, 5.0]) # 外乱を追加

        # 位置の更新: v [m/s] × dt [s] = 変位 [m]
        self.true_position += self.true_velocity * dt
