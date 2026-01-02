import csv
import json
import numpy as np
from typing import List, Dict, Optional, Tuple
from collections import defaultdict
import datetime


class DataLogger:
    """
    シミュレーション中のデータを収集し、CSVファイルに保存するクラス。
    """
    def __init__(self):
        self.timestamp: List[float] = []
        self.uav_trajectories: Dict[str, List[np.ndarray]] = defaultdict(list)
        self.fused_RL_errors_pair: Dict[str, List[float]] = defaultdict(list)
        self.inter_uav_distance_pair: Dict[str, List[float]] = defaultdict(list)
        self.relative_velocity_pair: Dict[str, List[float]] = defaultdict(list) # 実際は相対速度の大きさを保存
        self._creation_time = datetime.datetime.now()  # インスタンス生成時の時刻

    @staticmethod
    def _parse_pair_key(key: str) -> Tuple[int, int]:
        """キー uav<i>_<j>_fused_error から (i, j) を取り出す"""
        prefix = "uav"
        suffix = "_fused_error"
        if not (key.startswith(prefix) and key.endswith(suffix)):
            raise ValueError(f"Unexpected pair key format: {key}")
        body = key[len(prefix):-len(suffix)]
        parts = body.split("_")
        if len(parts) != 2:
            raise ValueError(f"Unexpected pair key format: {key}")
        return int(parts[0]), int(parts[1])

    @staticmethod
    def _pair_label(pair: Tuple[int, int]) -> str:
        """(i, j) を 'i-j' 形式の表示ラベルに変換する"""
        i, j = pair
        return f"{i}-{j}"

    def logging_timestamp(self, time: float):
        self.timestamp.append(time)

    def logging_uav_trajectories(self, uav_id: int, uav_position: np.ndarray):
        self.uav_trajectories[f"uav{uav_id}_true_pos"].append(uav_position.copy())

    def logging_fused_RL_error_pair(self, pair: Tuple[int, int], error: float):
        """重複のないUAVペア (i<j) の推定誤差をロギング"""
        i, j = sorted(pair)
        self.fused_RL_errors_pair[f"uav{i}_{j}_fused_error"].append(error)

    def logging_inter_uav_distance_pair(self, pair: Tuple[int, int], distance: float):
        """重複のないUAVペア (i<j) の機体間距離をロギング"""
        i, j = sorted(pair)
        self.inter_uav_distance_pair[f"dist_{i}_{j}"].append(distance)

    def logging_relative_velocity_pair(self, pair: Tuple[int, int], rel_velocity: np.ndarray):
        """重複のないUAVペア (i<j) の相対速度をロギング"""
        i, j = sorted(pair)
        self.relative_velocity_pair[f"rel_vel_{i}_{j}"].append(rel_velocity.copy())

    def calc_fused_RL_error_statistics(self, transient_time: float = 10.0) -> Dict[str, Dict[str, float]]:
        """
        融合推定誤差（ペア単位）の平均・分散・標準偏差などを算出する

        Args:
            transient_time (float): 過渡状態として除外する時間 [秒]（デフォルト: 10秒）

        Returns:
            Dict[str, Dict[str, float]]: キーは "uav<i>_<j>_fused_error"、値は統計量の辞書
        """
        statistics: Dict[str, Dict[str, float]] = {}

        # サンプリング周期を推定（最初の2つのタイムスタンプから）
        if len(self.timestamp) >= 2:
            dt = self.timestamp[1] - self.timestamp[0]
            transient_steps = int(transient_time / dt) if dt > 0 else 0
        else:
            transient_steps = 0

        for key in sorted(self.fused_RL_errors_pair.keys()):
            errors = self.fused_RL_errors_pair[key]
            stable_errors = [e for e in errors[transient_steps:] if e is not None and not np.isnan(e)]

            if stable_errors:
                mean_error = float(np.mean(stable_errors))
                variance = float(np.var(stable_errors))
                statistics[key] = {
                    'mean': mean_error,
                    'variance': variance,
                    'std': float(np.sqrt(variance)),
                    'num_samples': len(stable_errors)
                }
            else:
                statistics[key] = {
                    'mean': None,
                    'variance': None,
                    'std': None,
                    'num_samples': 0
                }

        return statistics

    def print_fused_RL_error_statistics(self, transient_time: float = 10.0):
        """融合推定誤差の統計情報をコンソールに表示する（ペアベース）"""
        statistics = self.calc_fused_RL_error_statistics(transient_time)

        print("\n" + "="*70)
        print(f"  融合RL推定誤差の統計 ({transient_time}秒後から安定状態)")
        print("="*70)
        print(f"{'UAV Pair':<10} | {'Mean Error (m)':<18} | {'Variance':<15} | {'Std Dev (m)':<15}")
        print("-" * 70)

        for key in sorted(statistics.keys()):
            stats = statistics[key]
            pair = self._parse_pair_key(key)
            label = self._pair_label(pair)
            if stats['mean'] is not None:
                print(f" {label:<9} | {stats['mean']:<18.6f} | {stats['variance']:<15.6f} | {stats['std']:<15.6f}")
            else:
                print(f" {label:<9} | {'N/A':<18} | {'N/A':<15} | {'N/A':<15}")

        print("="*70)
        return statistics

    def save_fused_RL_error_statistics(self, transient_time: float = 10.0,
                                       filename: Optional[str] = None,
                                       format: str = 'json') -> str:
        """
        融合推定誤差の統計情報を外部ファイルに保存する関数

        Args:
            transient_time (float): 過渡状態として除外する時間 [秒]
            filename (Optional[str]): 保存するファイル名（Noneの場合は自動生成）
            format (str): 保存形式 ('json' または 'txt')

        Returns:
            str: 保存されたファイルのパス
        """
        statistics = self.calc_fused_RL_error_statistics(transient_time)

        # ファイル名が指定されていない場合は自動生成
        if filename is None:
            timestamp_str = self._creation_time.strftime(r'%Y-%m-%d-%H-%M-%S')
            if format == 'json':
                filename = f'fused_RL_error_statistics_{timestamp_str}.json'
            else:
                filename = f'fused_RL_error_statistics_{timestamp_str}.txt'

        dir_path = f"../data/statistics/{format}/{filename}"

        if format == 'json':
            # JSON形式で保存
            # NumPy型をPython標準型に変換
            json_data = {
                'transient_time': transient_time,
                'timestamp': self._creation_time.strftime(r'%Y-%m-%d %H:%M:%S'),
                'statistics': {}
            }

            for key, stats in statistics.items():
                pair = self._parse_pair_key(key)
                json_data['statistics'][f'UAV_{pair[0]}_{pair[1]}'] = {
                    'mean': float(stats['mean']) if stats['mean'] is not None else None,
                    'variance': float(stats['variance']) if stats['variance'] is not None else None,
                    'std': float(stats['std']) if stats['std'] is not None else None,
                    'num_samples': int(stats['num_samples'])
                }

            with open(dir_path, 'w', encoding='utf-8') as f:
                json.dump(json_data, f, indent=4, ensure_ascii=False)

        else:  # txt形式で保存
            with open(dir_path, 'w', encoding='utf-8') as f:
                f.write("="*70 + "\n")
                f.write(f"  融合RL推定誤差の統計 ({transient_time}秒後から安定状態)\n")
                f.write(f"  生成日時: {self._creation_time.strftime(r'%Y-%m-%d %H:%M:%S')}\n")
                f.write("="*70 + "\n\n")
                f.write(f"{'UAV Pair':<10} | {'Mean Error (m)':<18} | {'Variance':<15} | {'Std Dev (m)':<15}\n")
                f.write("-" * 70 + "\n")

                for key, stats in sorted(statistics.items()):
                    pair = self._parse_pair_key(key)
                    label = self._pair_label(pair)
                    if stats['mean'] is not None:
                        f.write(f" {label:<9} | {stats['mean']:<18.6f} | {stats['variance']:<15.6f} | {stats['std']:<15.6f}\n")
                    else:
                        f.write(f" {label:<9} | {'N/A':<18} | {'N/A':<15} | {'N/A':<15}\n")

                f.write("="*70 + "\n")
                f.write(f"\nサンプル数:\n")
                for key, stats in sorted(statistics.items()):
                    pair = self._parse_pair_key(key)
                    label = self._pair_label(pair)
                    f.write(f"  UAV {label}: {stats['num_samples']} samples\n")

        print(f"Statistics successfully saved to {dir_path}")
        return dir_path

    def save_UAV_trajectories_data_to_csv(self, total_uav_num: int, filename: Optional[str] = None):
        """
        複数のUAVの軌道(2D)をcsv保存する関数

        Args:
            total_uav_num (int): UAVの全機体数
            filename (Optional[str]): 保存するCSVファイル名（Noneの場合は自動生成）
        """
        if filename is None:
            filename = f'uav_trajectories_{self._creation_time.strftime(r"%Y-%m-%d-%H-%M-%S")}.csv'

        dir_path = "../data/csv/trajectories/" + filename
        with open(dir_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write headers
            headers = ['time'] + [f'uav{i}_true_pos_x' for i in range(1, total_uav_num+1)] + [f'uav{i}_true_pos_y' for i in range(1, total_uav_num+1)]
            writer.writerow(headers)

            # Write data
            for t, positions in zip(self.timestamp, zip(*[self.uav_trajectories[f'uav{i}_true_pos'] for i in range(1, total_uav_num+1)])):
                row = [t] + [pos[0] for pos in positions] + [pos[1] for pos in positions]
                writer.writerow(row)
        print(f"Data successfully saved to {filename}")
        return filename

    def save_fused_RL_errors_to_csv(self, filename: Optional[str] = None):
        """
        相対自己位置の融合推定誤差をcsv保存する関数

        Args:
            filename (Optional[str]): 保存するCSVファイル名（Noneの場合は自動生成）
        """
        if filename is None:
            filename = f'fused_RL_error_{self._creation_time.strftime(r"%Y-%m-%d-%H-%M-%S")}.csv'

        dir_path = "../data/csv/RL_errors/" + filename
        with open(dir_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            if self.fused_RL_errors_pair:
                headers = ['time'] + sorted(self.fused_RL_errors_pair.keys())
                writer.writerow(headers)

                error_series = [self.fused_RL_errors_pair[key] for key in headers[1:]]
                for t, errors in zip(self.timestamp, zip(*error_series)):
                    row = [t] + list(errors)
                    writer.writerow(row)
        print(f"Data successfully saved to {filename}")
        return filename

    def save_inter_uav_distance_to_csv(self, filename: Optional[str] = None):
        """
        UAVの機体間距離をcsv保存する関数

        Args:
            filename (Optional[str]): 保存するCSVファイル名（Noneの場合は自動生成）
        """
        if filename is None:
            filename = f'inter_uav_distance_{self._creation_time.strftime(r"%Y-%m-%d-%H-%M-%S")}.csv'

        dir_path = "../data/csv/inter_uav_dist/" + filename
        with open(dir_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            if self.inter_uav_distance_pair:
                headers = ['time'] + sorted(self.inter_uav_distance_pair.keys())
                writer.writerow(headers)

                dist_series = [self.inter_uav_distance_pair[key] for key in headers[1:]]
                for t, dist in zip(self.timestamp, zip(*dist_series)):
                    row = [t] + list(dist)
                    writer.writerow(row)
        print(f"Data successfully saved to {filename}")
        return filename

    def save_relative_velocity_to_csv(self, filename: Optional[str] = None):
        """
        UAVの相対速度をcsv保存する関数

        Args:
            filename (Optional[str]): 保存するCSVファイル名（Noneの場合は自動生成）
        """
        if filename is None:
            filename = f'relative_velocity_{self._creation_time.strftime(r"%Y-%m-%d-%H-%M-%S")}.csv'

        dir_path = "../data/csv/relative_velocity/" + filename
        with open(dir_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            if self.relative_velocity_pair:
                headers = ['time'] + sorted(self.relative_velocity_pair.keys())
                writer.writerow(headers)

                vel_series = [self.relative_velocity_pair[key] for key in headers[1:]]
                for t, vel in zip(self.timestamp, zip(*vel_series)):
                    row = [t] + list(vel)
                    writer.writerow(row)
        print(f"Data successfully saved to {filename}")
        return filename

    def get_latest_trajectory_filename(self) -> str:
        """最新の軌道データファイル名を取得"""
        return f'uav_trajectories_{self._creation_time.strftime(r"%Y-%m-%d-%H-%M-%S")}.csv'

    def get_latest_error_filename(self) -> str:
        """最新のエラーデータファイル名を取得"""
        return f'fused_RL_error_{self._creation_time.strftime(r"%Y-%m-%d-%H-%M-%S")}.csv'
