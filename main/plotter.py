import pandas as pd
import matplotlib.pyplot as plt
import datetime
import re
from typing import Optional


class Plotter:
    """
    CSVファイルからデータを読み込んでグラフを生成するクラス
    """

    @staticmethod
    def plot_UAV_trajectories_from_csv(filename: str, total_uav_num: int, save_filename: Optional[str] = None):
        """
        複数のUAVの軌跡を2Dプロットする関数

        Args:
            filename (str): 読み込むCSVファイル名
            total_uav_num (int): UAVの全機体数
            save_filename (Optional[str]): 保存するグラフファイル名（Noneの場合は自動生成）
        """
        try:
            # Read CSV file
            file_path = f"../data/csv/trajectories/{filename}"
            data = pd.read_csv(file_path)

            plt.figure(figsize=(10, 8))
            for i in range(1, total_uav_num+1):
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

            # 保存ファイル名の生成
            if save_filename is None:
                timestamp_str = datetime.datetime.now().strftime(r'%Y-%m-%d-%H-%M-%S')
                save_filename = f'uav_trajectories_graph_{timestamp_str}.png'

            save_path = f'../data/graph/trajectories/{save_filename}'
            plt.savefig(save_path)
            print(f"Graph successfully saved to {save_path}")
            plt.show()

        except FileNotFoundError:
            print(f"Error: The file {filename} was not found.")
        except Exception as e:
            print(f"An error occurred while plotting: {e}")

    @staticmethod
    def plot_fused_RL_errors_from_csv(filename: str, save_filename: Optional[str] = None):
        """
        融合推定誤差をCSVファイルから読み込んでプロットする関数

        Args:
            filename (str): 読み込むCSVファイル名
            save_filename (Optional[str]): 保存するグラフファイル名（Noneの場合は自動生成）
        """
        try:
            file_path = f"../data/csv/RL_errors/{filename}"
            data = pd.read_csv(file_path)

            time_col = 'time'
            if time_col not in data.columns:
                raise KeyError("`time` column is missing in the error CSV")

            pair_pattern = re.compile(r"uav(\d+)_([\d]+)_fused_error")

            pair_cols = [c for c in data.columns if pair_pattern.fullmatch(c)]

            # RL保存の形式が正規表現に合わなければ例外を発生
            target_cols = pair_cols if pair_cols else None
            if not target_cols:
                raise ValueError("No fused error columns found in CSV")

            fig, ax = plt.subplots(figsize=(12, 6))
            color_cycle = plt.rcParams['axes.prop_cycle'].by_key().get('color', [])

            for idx, col in enumerate(sorted(target_cols)):
                errors = data[col]
                valid_mask = ~errors.isna()
                if not valid_mask.any():
                    continue

                times = data.loc[valid_mask, time_col]
                vals = errors[valid_mask]

                m_pair = pair_pattern.fullmatch(col)
                if m_pair:
                    i_id, j_id = m_pair.groups()
                    label = rf'$||\pi_{{{i_id}{j_id}}}(k) - \chi_{{{i_id}{j_id}}}(k)||$'

                color = color_cycle[idx % len(color_cycle)] if color_cycle else None
                ax.plot(times, vals, label=label, color=color)

            ax.set_title('Consensus-based RL Fusion Estimation', fontsize=16, fontweight='bold')
            ax.set_xlabel('$k$ (sec)', fontsize=14)
            ax.set_ylabel(r'$||\pi_{ij}(k) - \chi_{ij}(k)||$ (m)', fontsize=14)
            ax.grid(True)
            ax.legend()

            if save_filename is None:
                timestamp_str = datetime.datetime.now().strftime(r'%Y-%m-%d-%H-%M-%S')
                save_filename = f'fused_RL_errors_graph_{timestamp_str}.png'

            save_path = f'../data/graph/RL_errors/{save_filename}'
            plt.savefig(save_path)
            print(f"Graph successfully saved to {save_path}")

            plt.show()

        except FileNotFoundError:
            print(f"Error: The file {filename} was not found.")
        except Exception as e:
            print(f"An error occurred while plotting: {e}")
