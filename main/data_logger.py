import csv
import json
import numpy as np
from typing import List, Dict, Optional
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
        self._creation_time = datetime.datetime.now()  # インスタンス生成時の時刻

    def logging_timestamp(self, time: float):
        self.timestamp.append(time)

    def logging_uav_trajectories(self, uav_id: int, uav_position: np.ndarray):
        self.uav_trajectories[f"uav{uav_id}_true_pos"].append(uav_position.copy())

    def logging_fused_RL_error_pair(self, pair: tuple[int, int], error: float):
        """重複のないUAVペア (i<j) の推定誤差をロギング"""
        i, j = sorted(pair)
        self.fused_RL_errors_pair[f"uav{i}_{j}_fused_error"].append(error)

    def calc_fused_RL_error_statistics(self, total_uav_num: int, transient_time: float = 10.0) -> Dict[int, Dict[str, float]]:
        """
        各UAVの融合推定誤差の平均と分散を計算する関数

        Args:
            transient_time (float): 過渡状態として除外する時間 [秒]（デフォルト: 10秒）
            total_uav_num (int): UAVの全機体数

        Returns:
            Dict[int, Dict[str, float]]: UAV IDをキーとし、'mean'と'variance'を含む辞書
                例: {1: {'mean': 0.123, 'variance': 0.456}, 2: {...}, ...}
        """
        statistics = {}

        # サンプリング周期を推定（最初の2つのタイムスタンプから）
        if len(self.timestamp) >= 2:
            dt = self.timestamp[1] - self.timestamp[0]
            transient_steps = int(transient_time / dt)
        else:
            transient_steps = 0

        # UAVの推定誤差について統計を計算
        for uav_id in range(total_uav_num):
            key = f"uav{uav_id}_fused_error"
            if key in self.fused_RL_errors:
                errors = self.fused_RL_errors[key]

                # 過渡状態を除外し、有効な誤差のみを抽出
                stable_errors = [e for e in errors[transient_steps:] if e is not None and not np.isnan(e)]

                if stable_errors:
                    mean_error = np.mean(stable_errors)
                    variance = np.var(stable_errors)
                    statistics[uav_id] = {
                        'mean': mean_error,
                        'variance': variance,
                        'std': np.sqrt(variance),
                        'num_samples': len(stable_errors)
                    }
                else:
                    statistics[uav_id] = {
                        'mean': None,
                        'variance': None,
                        'std': None,
                        'num_samples': 0
                    }

        return statistics

    def print_fused_RL_error_statistics(self, total_uav_num: int, transient_time: float = 10.0):
        """
        融合推定誤差の統計情報をコンソールに表示する関数

        Args:
            transient_time (float): 過渡状態として除外する時間 [秒]
            total_uav_num (int): UAVの全機体数
        """
        statistics = self.calc_fused_RL_error_statistics(total_uav_num, transient_time)

        print("\n" + "="*70)
        print(f"  融合RL推定誤差の統計 ({transient_time}秒後から安定状態)")
        print("="*70)
        print(f"{'UAV Pair':<10} | {'Mean Error (m)':<18} | {'Variance':<15} | {'Std Dev (m)':<15}")
        print("-" * 70)

        for uav_id in range(total_uav_num):
            if uav_id in statistics:
                stats = statistics[uav_id]
                if stats['mean'] is not None:
                    print(f" {uav_id}→1    | {stats['mean']:<18.6f} | {stats['variance']:<15.6f} | {stats['std']:<15.6f}")
                else:
                    print(f" {uav_id}→1    | {'N/A':<18} | {'N/A':<15} | {'N/A':<15}")

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

            for uav_id, stats in statistics.items():
                json_data['statistics'][f'UAV_{uav_id}_to_1'] = {
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

                for uav_id in range(2, 7):
                    if uav_id in statistics:
                        stats = statistics[uav_id]
                        if stats['mean'] is not None:
                            f.write(f" {uav_id}→1    | {stats['mean']:<18.6f} | {stats['variance']:<15.6f} | {stats['std']:<15.6f}\n")
                        else:
                            f.write(f" {uav_id}→1    | {'N/A':<18} | {'N/A':<15} | {'N/A':<15}\n")

                f.write("="*70 + "\n")
                f.write(f"\nサンプル数:\n")
                for uav_id in range(2, 7):
                    if uav_id in statistics:
                        f.write(f"  UAV {uav_id}→1: {statistics[uav_id]['num_samples']} samples\n")

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
                # ペア単位でのログが存在する場合はこちらを優先
                headers = ['time'] + sorted(self.fused_RL_errors_pair.keys())
                writer.writerow(headers)

                error_series = [self.fused_RL_errors_pair[key] for key in headers[1:]]
                for t, errors in zip(self.timestamp, zip(*error_series)):
                    row = [t] + list(errors)
                    writer.writerow(row)
        print(f"Data successfully saved to {filename}")
        return filename

    def get_latest_trajectory_filename(self) -> str:
        """最新の軌道データファイル名を取得"""
        return f'uav_trajectories_{self._creation_time.strftime(r"%Y-%m-%d-%H-%M-%S")}.csv'

    def get_latest_error_filename(self) -> str:
        """最新のエラーデータファイル名を取得"""
        return f'fused_RL_error_{self._creation_time.strftime(r"%Y-%m-%d-%H-%M-%S")}.csv'
