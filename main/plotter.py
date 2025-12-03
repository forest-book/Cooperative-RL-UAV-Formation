import pandas as pd
import matplotlib.pyplot as plt
import datetime
from typing import Optional


class Plotter:
    """
    CSVファイルからデータを読み込んでグラフを生成するクラス
    """
    
    @staticmethod
    def plot_UAV_trajectories_from_csv(filename: str, save_filename: Optional[str] = None):
        """
        複数のUAVの軌跡を2Dプロットする関数

        Args:
            filename (str): 読み込むCSVファイル名
            save_filename (Optional[str]): 保存するグラフファイル名（Noneの場合は自動生成）
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
                valid_times = data['time'][~errors.isna()]
                valid_errors = errors[~errors.isna()]
                if not valid_errors.empty:
                    axins.plot(valid_times, valid_errors, color=colors.get(i, 'k'))
            axins.set_xlim(98, 110)  # 論文のズーム範囲に合わせる
            axins.set_ylim(0, 0.8)
            axins.grid(True)
            ax.indicate_inset_zoom(axins, edgecolor="black")  # ズーム箇所を四角で表示

            # 保存ファイル名の生成
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
