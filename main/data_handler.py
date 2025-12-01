import csv
import json
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
from typing import List, Dict, Optional
import datetime

# データ保存時の日時
current_time = datetime.datetime.now()

class DataLogger:
    """
    シミュレーション中のデータを収集し、CSVファイルに保存するクラス。
    """
    def __init__(self):
        self.timestamp: List[float] = []
        self.uav_trajectories: Dict[str, List[np.ndarray]] = defaultdict(list)
        self.fused_RL_errors: Dict[str, List[float]] = defaultdict(list)

    def logging_timestamp(self, time: float):
        self.timestamp.append(time)

    def logging_uav_trajectories(self, uav_id: int, uav_position: np.ndarray):
        self.uav_trajectories[f"uav{uav_id}_true_pos"].append(uav_position.copy())

    def logging_fused_RL_error(self, uav_id: int, error: float):
        self.fused_RL_errors[f"uav{uav_id}_fused_error"].append(error)

    def calc_fused_RL_error_statistics(self, transient_time: float = 10.0) -> Dict[int, Dict[str, float]]:
        """
        各UAVの融合推定誤差の平均と分散を計算する関数
        
        Args:
            transient_time (float): 過渡状態として除外する時間 [秒]（デフォルト: 10秒）
            
        Returns:
            Dict[int, Dict[str, float]]: UAV IDをキーとし、'mean'と'variance'を含む辞書
                例: {2: {'mean': 0.123, 'variance': 0.456}, 3: {...}, ...}
        """
        statistics = {}
        
        # サンプリング周期を推定（最初の2つのタイムスタンプから）
        if len(self.timestamp) >= 2:
            dt = self.timestamp[1] - self.timestamp[0]
            transient_steps = int(transient_time / dt)
        else:
            transient_steps = 0
        
        # UAV 2~6 の誤差について統計を計算
        for uav_id in range(2, 7):
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

    def print_fused_RL_error_statistics(self, transient_time: float = 10.0):
        """
        融合推定誤差の統計情報をコンソールに表示する関数
        
        Args:
            transient_time (float): 過渡状態として除外する時間 [秒]
        """
        statistics = self.calc_fused_RL_error_statistics(transient_time)
        
        print("\n" + "="*70)
        print(f"  融合RL推定誤差の統計 ({transient_time}秒後から安定状態)")
        print("="*70)
        print(f"{'UAV Pair':<10} | {'Mean Error (m)':<18} | {'Variance':<15} | {'Std Dev (m)':<15}")
        print("-" * 70)
        
        for uav_id in range(2, 7):
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
            filename (str): 保存するファイル名（Noneの場合は自動生成）
            format (str): 保存形式 ('json' または 'txt')
            
        Returns:
            str: 保存されたファイルのパス
        """
        statistics = self.calc_fused_RL_error_statistics(transient_time)
        
        # ファイル名が指定されていない場合は自動生成
        if filename is None:
            timestamp_str = current_time.strftime(r'%Y-%m-%d-%H-%M-%S')
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
                'timestamp': current_time.strftime(r'%Y-%m-%d %H:%M:%S'),
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
                f.write(f"  生成日時: {current_time.strftime(r'%Y-%m-%d %H:%M:%S')}\n")
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

    def save_UAV_trajectories_data_to_csv(self, filename: str = f'uav_trajectories_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.csv'):
        """
        複数のUAVの軌道(2D)をcsv保存する関数

        Args:
            filename (str): 保存するCSVファイル名
        """
        dir_path = "../data/csv/trajectories/" + filename
        with open(dir_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write headers
            headers = ['time'] + [f'uav{i}_true_pos_x' for i in range(1, 7)] + [f'uav{i}_true_pos_y' for i in range(1, 7)]
            writer.writerow(headers)

            # Write data
            for t, positions in zip(self.timestamp, zip(*[self.uav_trajectories[f'uav{i}_true_pos'] for i in range(1, 7)])):
                row = [t] + [pos[0] for pos in positions] + [pos[1] for pos in positions]
                writer.writerow(row)
        print(f"Data successfully saved to {filename}")

    def save_fused_RL_errors_to_csv(self, filename: str = f'fused_RL_error_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.csv'):
        """
        相対自己位置の融合推定誤差をcsv保存する関数

        Args:
            filename (str): 保存するCSVファイル名
        """
        dir_path = "../data/csv/RL_errors/" + filename
        with open(dir_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Write headers
            headers = ['time'] + [f'uav{i}_fused_error' for i in range(2, 7)]
            writer.writerow(headers)

            # Write data
            for t, errors in zip(self.timestamp, zip(*[self.fused_RL_errors[f'uav{i}_fused_error'] for i in range(2, 7)])):
                row = [t] + list(errors)
                writer.writerow(row)
        print(f"Data successfully saved to {filename}")

class Plotter:
    @staticmethod
    def plot_UAV_trajectories_from_csv(filename: str = f'uav_trajectories_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.csv'):
        """
        複数のUAVの軌跡を2Dプロットする関数

        Args:
            filename (str): 読み込むCSVファイル名
        """
        try:
            # Read CSV file
            file_path = f"../data/csv/trajectories/{filename}"
            data = pd.read_csv(file_path)

            plt.figure(figsize=(10, 8))
            for i in range(1, 7):
                x_positions = data[f'uav{i}_true_pos_x']
                y_positions = data[f'uav{i}_true_pos_y']
                plt.plot(x_positions, y_positions, label=f'UAV {i}')
                plt.scatter(x_positions.iloc[0], y_positions.iloc[0], marker='o', label=f'UAV {i} Start')
                plt.scatter(x_positions.iloc[-1], y_positions.iloc[-1], marker='x', label=f'UAV {i} End')

            plt.title('UAV Trajectories from CSV')
            plt.xlabel('X position (m)')
            plt.ylabel('Y position (m)')
            plt.legend()
            plt.grid(True)
            plt.axis('equal')
            plt.savefig(f'../data/graph/trajectories/uav_trajectories_graph_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.png')
            print(f"Graph successfully saved to uav_trajectories_graph_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.png")
            plt.show()

        except FileNotFoundError:
            print(f"Error: The file {filename} was not found.")
        except Exception as e:
            print(f"An error occurred while plotting: {e}")

    @staticmethod
    def plot_fused_RL_errors_from_csv(filename: str = f'fused_RL_error_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.csv'):
        """
        Plot fusion estimation errors from a CSV file.

        Parameters:
            filename (str): Name of the CSV file to read.
        """

        # Read CSV file
        file_path = f"../data/csv/RL_errors/{filename}"
        data = pd.read_csv(file_path)

        fig, ax = plt.subplots(figsize=(12, 6))
        colors = {2: 'c', 3: 'b', 4: 'g', 5: 'r', 6: 'm'}

        for i in range(2, 7):
            errors = data[f'uav{i}_fused_error']
            valid_times = data['time'][~errors.isna()]
            valid_errors = errors[~errors.isna()]

            if not valid_errors.empty:
                ax.plot(valid_times, valid_errors, 
                        label=rf'$||\pi_{{{i}1}} - \chi_{{{i}1}}||$', 
                        color=colors.get(i, 'k'))

        ax.set_title('Consensus-based RL Fusion Estimation', fontsize=16, fontweight='bold')
        ax.set_xlabel('$k$ (sec)', fontsize=14)
        ax.set_ylabel(r'$||\pi_{ij}(k) - \chi_{ij}(k)||$ (m)', fontsize=14)
        ax.set_ylim(0, 50.0)
        ax.legend()
        ax.grid(True)

        # 図4(e)のズームインした図を挿入
        axins = ax.inset_axes((0.5, 0.5, 0.4, 0.4))
        for i in range(2, 7):
             errors = data[f'uav{i}_fused_error']
             valid_times = [t for t, e in zip(data['time'], errors) if e is not None]
             valid_errors = [e for e in errors if e is not None]
             if valid_errors:
                axins.plot(valid_times, valid_errors, color=colors.get(i, 'k'))
        axins.set_xlim(100, 150) # 論文のズーム範囲に合わせる
        axins.set_ylim(0, 0.6)
        axins.grid(True)
        ax.indicate_inset_zoom(axins, edgecolor="black") # ズーム箇所を四角で表示

        #todo グラフズーム範囲の修正
        plt.savefig(f'../data/graph/RL_errors/fused_RL_errors_graph_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.png')
        print(f"Graph successfully saved to fused_RL_errors_graph_{current_time.strftime(r'%Y-%m-%d-%H-%M-%S')}.png")
        
        plt.show()

