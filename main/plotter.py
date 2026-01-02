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
    def _read_csv_and_validate_time(file_path: str, data_type: str) -> pd.DataFrame:
        """
        CSVファイルを読み込み、time列の存在を検証する共通ヘルパー関数

        Args:
            file_path (str): CSVファイルのパス
            data_type (str): データの種類（エラーメッセージ用）

        Returns:
            pd.DataFrame: 読み込んだデータフレーム

        Raises:
            KeyError: time列が存在しない場合
        """
        data = pd.read_csv(file_path)
        time_col = 'time'
        if time_col not in data.columns:
            raise KeyError(f"`time` column is missing in the {data_type} CSV")
        return data

    @staticmethod
    def _extract_columns_by_pattern(data: pd.DataFrame, pattern: re.Pattern, data_type: str) -> list:
        """
        正規表現パターンに基づいて列を抽出する共通ヘルパー関数

        Args:
            data (pd.DataFrame): データフレーム
            pattern (re.Pattern): 列名のマッチング用正規表現パターン
            data_type (str): データの種類（エラーメッセージ用）

        Returns:
            list: マッチした列名のリスト

        Raises:
            ValueError: マッチする列が見つからない場合
        """
        matched_cols = [col for col in data.columns if pattern.fullmatch(col)]
        if not matched_cols:
            raise ValueError(f"No {data_type} columns found in CSV")
        return matched_cols

    @staticmethod
    def _plot_time_series_data(data: pd.DataFrame, target_cols: list, pattern: re.Pattern,
                               label_formatter: callable, title: str, xlabel: str, ylabel: str,
                               additional_plot_func: callable = None) -> tuple:
        """
        時系列データをプロットする共通ヘルパー関数

        Args:
            data (pd.DataFrame): データフレーム
            target_cols (list): プロットする列のリスト
            pattern (re.Pattern): 列名のマッチング用正規表現パターン
            label_formatter (callable): ラベル生成関数（i_id, j_idを引数に取る）
            title (str): グラフのタイトル
            xlabel (str): X軸ラベル
            ylabel (str): Y軸ラベル
            additional_plot_func (callable, optional): 追加のプロット処理を行う関数

        Returns:
            tuple: (fig, ax) Matplotlibのfigureとaxesオブジェクト
        """
        fig, ax = plt.subplots(figsize=(12, 6))
        color_cycle = plt.rcParams['axes.prop_cycle'].by_key().get('color', [])
        time_col = 'time'

        for idx, col in enumerate(sorted(target_cols)):
            values = data[col]
            valid_mask = ~values.isna()
            if not valid_mask.any():
                continue

            times = data.loc[valid_mask, time_col]
            vals = values[valid_mask]

            label = col  # Default to column name
            m_pair = pattern.fullmatch(col)
            if m_pair:
                i_id, j_id = m_pair.groups()
                label = label_formatter(i_id, j_id)

            color = color_cycle[idx % len(color_cycle)] if color_cycle else None
            ax.plot(times, vals, label=label, color=color)

        # 追加のプロット処理がある場合は実行
        if additional_plot_func:
            additional_plot_func(ax)

        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.set_xlabel(xlabel, fontsize=14)
        ax.set_ylabel(ylabel, fontsize=14)
        ax.grid(True)
        ax.legend()

        return fig, ax

    @staticmethod
    def _save_figure(save_path: str):
        """
        Matplotlibのfigureを保存する共通ヘルパー関数

        Args:
            save_path (str): 保存先のパス
        """
        plt.savefig(save_path)
        print(f"Graph successfully saved to {save_path}")
        plt.show()

    @staticmethod
    def _generate_save_path(base_dir: str, save_filename: Optional[str], default_prefix: str) -> str:
        """
        保存ファイル名とパスを生成する共通ヘルパー関数

        Args:
            base_dir (str): ベースディレクトリ
            save_filename (Optional[str]): 保存するファイル名（Noneの場合は自動生成）
            default_prefix (str): デフォルトのファイル名プレフィックス

        Returns:
            str: 保存先のフルパス
        """
        if save_filename is None:
            timestamp_str = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
            save_filename = f'{default_prefix}_{timestamp_str}.png'
        return f'{base_dir}/{save_filename}'

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

    @staticmethod
    def plot_inter_uav_distance_from_csv(filename: str, save_filename: Optional[str] = None):
        """
        UAV機体間距離をCSVファイルから読み込んでプロットする関数

        Args:
            filename (str): 読み込むCSVファイル名
            save_filename (Optional[str]): 保存するグラフファイル名（Noneの場合は自動生成）
        """
        try:
            file_path = f"../data/csv/inter_uav_dist/{filename}"
            data = Plotter._read_csv_and_validate_time(file_path, "inter-uav distance")

            pair_pattern = re.compile(r"dist_(\d+)_(\d+)")
            target_cols = Plotter._extract_columns_by_pattern(data, pair_pattern, "inter-uav distance")

            def label_formatter(i_id, j_id):
                return rf'$d^{{{i_id}{j_id}}}$'

            def add_distance_guidelines(ax):
                """目標距離のガイドライン (15m, 30m) を追加"""
                ax.axhline(y=15, color='gray', linestyle=':', linewidth=1.5, alpha=0.8)  # Target 15m
                ax.axhline(y=30, color='gray', linestyle=':', linewidth=1.5, alpha=0.8)  # Target 30m
                ax.text(0.5, 15.5, 'Target: 15m', fontsize=10, color='gray')
                ax.text(0.5, 30.5, 'Target: 30m', fontsize=10, color='gray')

            Plotter._plot_time_series_data(
                data, target_cols, pair_pattern, label_formatter,
                'Inter-UAV Distance: $d^{ij}$',
                'Time (sec)',
                r'Distance (m)',
                add_distance_guidelines
            )

            save_path = Plotter._generate_save_path(
                '../data/graph/inter_uav_dist',
                save_filename,
                'inter_uav_distance_graph'
            )
            Plotter._save_figure(save_path)

        except FileNotFoundError:
            print(f"Error: The file {filename} was not found.")
        except Exception as e:
            print(f"An error occurred while plotting: {e}")

    @staticmethod
    def plot_relative_velocity_from_csv(filename: str, save_filename: Optional[str] = None):
        """
        UAV機体間の相対速度をCSVファイルから読み込んでプロットする関数

        Args:
            filename (str): 読み込むCSVファイル名
            save_filename (Optional[str]): 保存するグラフファイル名（Noneの場合は自動生成）
        """
        try:
            file_path = f"../data/csv/relative_velocity/{filename}"
            data = Plotter._read_csv_and_validate_time(file_path, "relative velocity")

            pair_pattern = re.compile(r"rel_vel_(\d+)_(\d+)")
            target_cols = Plotter._extract_columns_by_pattern(data, pair_pattern, "relative velocity")

            def label_formatter(i_id, j_id):
                return rf'$||v^{{{i_id}{j_id}}}||$'

            Plotter._plot_time_series_data(
                data, target_cols, pair_pattern, label_formatter,
                'Inter-UAV Relative Velocity Magnitude',
                'Time (sec)',
                r'Relative Velocity Magnitude (m/s)'
            )

            save_path = Plotter._generate_save_path(
                '../data/graph/relative_velocity',
                save_filename,
                'relative_velocity_graph'
            )
            Plotter._save_figure(save_path)

        except FileNotFoundError:
            print(f"Error: The file {filename} was not found.")
        except Exception as e:
            print(f"An error occurred while plotting: {e}")
