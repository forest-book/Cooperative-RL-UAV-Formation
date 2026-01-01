import numpy as np
from typing import List

from quadcopter import UAV
from estimator import Estimator
from data_logger import DataLogger
from plotter import Plotter
from config_loader import ConfigLoader
from interface_sensor import ISensor
from control_input import ControlInput

class MainController:
    """アプリケーション全体を管理し，メインループを実行する"""
    def __init__(self, params: dict, sensor: ISensor):
        self.params = params
        self.uavs: List[UAV] = []
        self.loop_amount: int = 0
        self.dt = params['T']   # サンプリング周期

        self.estimator = Estimator()
        self.data_logger = DataLogger()
        self.controller = ControlInput()
        self.sensor = sensor

    def get_uav_by_id(self, uav_id: int) -> UAV:
        """UAV IDからUAVオブジェクトを取得するヘルパーメソッド"""
        if uav_id < 1 or uav_id > len(self.uavs):
            raise ValueError(f"Invalid UAV ID: {uav_id}. Must be between 1 and {len(self.uavs)}")
        return self.uavs[uav_id - 1]

    @staticmethod
    def make_direct_estimate_key(from_uav_id: int, to_uav_id: int) -> str:
        """直接推定値の辞書キーを生成する"""
        return f"chi_{from_uav_id}_{to_uav_id}"

    @staticmethod
    def make_fused_estimate_key(from_uav_id: int, to_uav_id: int) -> str:
        """融合推定値の辞書キーを生成する"""
        return f"pi_{from_uav_id}_{to_uav_id}"

    def initialize_direct_estimates(self):
        """直接推定値の初期化（乱数付与）"""
        noise_bound = self.params['NOISE']['initialization_bound']  # 一様乱数の範囲を設定
        for uav in self.uavs:
            for neighbor_id in uav.neighbors:
                neighbor_uav = self.get_uav_by_id(neighbor_id)
                true_initial_rel_pos = neighbor_uav.true_position - uav.true_position
                # 一様乱数を生成して真値に加算
                noise = np.random.uniform(-noise_bound, noise_bound, size=true_initial_rel_pos.shape)
                noisy_initial_rel_pos = true_initial_rel_pos + noise
                key = self.make_direct_estimate_key(uav.id, neighbor_id)
                uav.direct_estimates[key].append(noisy_initial_rel_pos.copy())

    def initialize_fused_estimates(self):
        """融合推定値の初期化（乱数付与）"""
        noise_bound = self.params['NOISE']['initialization_bound']  # 一様乱数の範囲を設定
        for uav_i in self.uavs:
            # 自機以外のすべてのUAVに対して推定
            for target_j_uav in self.uavs:
                if target_j_uav.id == uav_i.id:
                    continue # 自機への推定は行わない
                true_initial_rel_pos: np.ndarray = target_j_uav.true_position - uav_i.true_position
                # 一様乱数を生成して真値に加算
                noise = np.random.uniform(-noise_bound, noise_bound, size=true_initial_rel_pos.shape)
                noisy_initial_rel_pos = true_initial_rel_pos + noise
                key = self.make_fused_estimate_key(uav_i.id, target_j_uav.id)
                uav_i.fused_estimates[key].append(noisy_initial_rel_pos.copy())

    def initialize_uav_setting(self):
        # UAVインスタンス化と初期位置・隣接機の設定をまとめて行う
        initial_positions: dict = self.params['INITIAL_POSITIONS']
        neighbors_setting: dict = self.params['NEIGHBORS']

        self.uavs.clear()  # 明示的にリセットしてから生成
        for uav_id, position in initial_positions.items():
            neighbors = neighbors_setting.get(uav_id, [])
            self.uavs.append(
                UAV(
                    uav_id=uav_id,
                    initial_position=position,
                    neighbors=neighbors,
                )
            )

    def initialize(self):
        """システムの初期化"""
        print("initialize simulation settings...")
        # UAVインスタンス化と初期位置・隣接機の設定
        self.initialize_uav_setting()

        # k=0での直接推定値を設定(直接推定値の初期化)
        # 隣接機に対してのみ初期化
        self.initialize_direct_estimates()

        # k=0での融合推定値を設定(融合推定値の初期化)
        # UAV_i(自機)から見たUAV_j(自機以外のすべてのUAV)の相対位置を融合推定
        self.initialize_fused_estimates()

        # 推定式はステップk(自然数)毎に状態を更新するため
        self.loop_amount = int(self.params['DURATION'] / self.params['T'])

    def calc_RL_estimation_error(self, uav_i_id: int, target_j_id: int, loop_num: int) -> float:
        # 真の相対位置
        target_uav = self.get_uav_by_id(target_j_id)
        uav_i = self.get_uav_by_id(uav_i_id)
        true_rel_pos = target_uav.true_position - uav_i.true_position
        # 相対位置の融合推定値
        key = self.make_fused_estimate_key(uav_i_id, target_j_id)
        estimate_rel_pos = uav_i.fused_estimates[key]
        # 推定誤差
        estimation_error = estimate_rel_pos[loop_num] - true_rel_pos
        # ノルムをとって推定誤差を距離に直す
        estimation_error_distance = np.linalg.norm(estimation_error)
        return estimation_error_distance

    def build_measurements_cache(self) -> dict:
        """全UAVペア間の測定値を事前計算してキャッシュする"""
        measurements_cache = {}
        delta_bar = self.params['NOISE']['delta_bar']
        dist_bound = self.params['NOISE']['dist_bound']
        for uav_i in self.uavs:
            for uav_j in self.uavs:
                if uav_i.id == uav_j.id:
                    continue
                key = (uav_i.id, uav_j.id)
                noisy_v = self.sensor.get_velocity_info(uav_i=uav_i, uav_j=uav_j, delta_bar=delta_bar, add_vel_noise=False)
                noisy_d = self.sensor.get_distance_info(uav_i=uav_i, uav_j=uav_j, dist_bound=dist_bound, add_dist_noise=False)
                noisy_d_dot = self.sensor.get_distance_rate_info(uav_i=uav_i, uav_j=uav_j, dist_bound=dist_bound, add_dist_rate_noise=False)
                measurements_cache[key] = (noisy_v, noisy_d, noisy_d_dot)
        return measurements_cache

    def get_neighbor_relative_velocities(self, uav: UAV, measurements_cache: dict) -> List[np.ndarray]:
        """隣接機との現在の相対速度ベクトルのリストを取得する"""
        relative_velocities: List[np.ndarray] = []
        for neighbor_id in uav.neighbors:
            key = (uav.id, neighbor_id)
            if key not in measurements_cache:
                raise KeyError(f"No cached measurement for UAV pair {key}")
            noisy_v, _, _ = measurements_cache[key]
            relative_velocities.append(noisy_v.copy())
        return relative_velocities

    def get_neighbor_relative_distances(self, uav: UAV, measurements_cache: dict) -> List[float]:
        """隣接機との現在の相対距離のリストを取得する"""
        relative_distances: List[float] = []
        for neighbor_id in uav.neighbors:
            key = (uav.id, neighbor_id)
            if key not in measurements_cache:
                raise KeyError(f"No cached measurement for UAV pair {key}")
            _, noisy_d, _ = measurements_cache[key]
            relative_distances.append(noisy_d)
        return relative_distances

    def get_neighbor_fused_RLs(self, uav: UAV, loop: int) -> List[np.ndarray]:
        """隣接機への融合RL推定位置ベクトルのリストを取得する"""
        fused_RLs: List[np.ndarray] = []
        for neighbor_id in uav.neighbors:
            key = self.make_fused_estimate_key(uav.id, neighbor_id)
            fused_RLs.append(uav.fused_estimates[key][loop].copy())
        return fused_RLs

    def exec_direct_estimation(self, measurements_cache: dict, loop: int) -> None:
        """直接推定の1ステップを実行する"""
        for uav_i in self.uavs:
            for neighbor_id in uav_i.neighbors:
                #print(f"uav_{uav_i.id}_{neighbor_id}")
                # キャッシュからノイズ付き観測値を取得
                noisy_v, noisy_d, noisy_d_dot = measurements_cache[(uav_i.id, neighbor_id)]
                # 式(1)の計算
                key = self.make_direct_estimate_key(uav_i.id, neighbor_id)
                chi_hat_ij_i_k = uav_i.direct_estimates[key] # k=loopの時の直接推定値を持ってくる

                next_direct = self.estimator.calc_direct_RL_estimate(
                    chi_hat_ij_i_k=chi_hat_ij_i_k[loop],
                    noisy_v=noisy_v,
                    noisy_d=noisy_d,
                    noisy_d_dot=noisy_d_dot,
                    T=self.dt,
                    gamma=self.params['GAMMA']
                ) # 次のステップ(k=loop + 1)の時の相対位置を直接推定
                # uav_iは直接推定値を持っている
                uav_i.direct_estimates[key].append(next_direct.copy())
                #print(uav_i.direct_estimates)

    def exec_indirect_estimation(self, uav_i: UAV, target_j_uav: UAV, loop: int) -> List[np.ndarray]:
        """事実上の間接推定の1ステップを実行する"""
        # 間接推定値のリストを作成
        indirect_estimates_list: List[np.ndarray] = []
        for r_id in uav_i.neighbors:
            if r_id == target_j_uav.id: # r(間接機)はtarget(推定対象)であってはならない
                continue
            uav_r = self.get_uav_by_id(r_id) #uav_iの隣接機UAVオブジェクト
            # uav_i(自機)からuav_r(間接機)への直接推定値
            direct_key_ir = self.make_direct_estimate_key(uav_i.id, uav_r.id)
            chi_hat_ir_i_k = uav_i.direct_estimates[direct_key_ir]
            # uav_r(間接機)からtarget(推定対象)への融合推定値
            fused_key_rj = self.make_fused_estimate_key(uav_r.id, target_j_uav.id)
            pi_rj_r_k = uav_r.fused_estimates[fused_key_rj]
            # uav_i(自機)からtarget(推定対象)への間接推定値
            chi_hat_ij_r_k: np.ndarray = chi_hat_ir_i_k[loop] + pi_rj_r_k[loop]
            # リストに格納
            indirect_estimates_list.append(chi_hat_ij_r_k.copy())
        return indirect_estimates_list

    def exec_fused_estimation(self, measurements_cache: dict, loop: int) -> None:
        """融合推定の1ステップを実行する"""
        # 自機以外のすべてのUAVに対する融合推定値を算出する
        for uav_i in self.uavs:
            for target_j_uav in self.uavs:
                #print(f"uav_{uav_i.id}_{target_j_uav.id}")
                # 自機以外のすべてのUAVの内1機をtargetとして推定するのを繰り返す
                if target_j_uav.id == uav_i.id:
                    continue  # 自身への推定は行わない

                # 重みκを計算
                kappa_D, kappa_I = self.estimator.calc_estimation_kappa(uav_i.neighbors.copy(), target_j_uav.id) # Listは参照渡しなのでcopyを渡す
                # キャッシュからノイズ付き相対速度 v_ij を取得
                noisy_v_ij, _, _ = measurements_cache[(uav_i.id, target_j_uav.id)]

                # 直接推定値と融合推定値を持ってくる
                direct_key = self.make_direct_estimate_key(uav_i.id, target_j_uav.id)
                fused_key = self.make_fused_estimate_key(uav_i.id, target_j_uav.id)
                chi_hat_ij_i_k = uav_i.direct_estimates[direct_key] # k=loopの時の直接推定値を持ってくる
                pi_ij_i_k = uav_i.fused_estimates[fused_key]

                # 間接推定値のリストを作成
                indirect_estimates_list = self.exec_indirect_estimation(uav_i, target_j_uav, loop)

                next_fused = self.estimator.calc_fused_RL_estimate(
                    pi_ij_i_k=pi_ij_i_k[loop],
                    direct_estimate_x_hat=chi_hat_ij_i_k[loop] if kappa_D != 0 else np.zeros(2),
                    indirect_estimates=indirect_estimates_list,
                    noisy_v=noisy_v_ij,
                    T=self.dt,
                    kappa_D=kappa_D,
                    kappa_I=kappa_I,
                ) # 次のステップ(k=loop + 1)の時の相対位置を融合推定

                uav_i.fused_estimates[fused_key].append(next_fused.copy())
                #print(uav_i.fused_estimates)

    def apply_control_input(self, measurements_cache, loop):
        """次のステップの制御入力（速度）を算出し適用する"""
        for uav_i in self.uavs:
            rel_velocities: List[np.ndarray] = self.get_neighbor_relative_velocities(uav_i, measurements_cache)
            rel_distances: List[float] = self.get_neighbor_relative_distances(uav_i, measurements_cache)
            desired_distance = self.params['DIST']
            fused_RLs: List[np.ndarray] = self.get_neighbor_fused_RLs(uav_i, loop)
            next_velocity = self.controller.calc_RL_based_control_input(
                vel_i_k=uav_i.true_velocity,
                rel_v_ij_i_k=rel_velocities,
                rel_distances=rel_distances,
                desired_distances=desired_distance,
                pi_ij_i_k=fused_RLs,
                T=self.dt,
                gamma1=self.params['GAMMA1'],
                gamma2=self.params['GAMMA2']
            )
            print(f"uav_{uav_i.id}の制御入力: {next_velocity}")
            uav_i.control_input = next_velocity
        return

    def show_simulation_progress(self, loop):
        if(loop * 100 // self.loop_amount) > ((loop - 1) *100 // self.loop_amount):
            print(f"simulation progress: {loop *100 // self.loop_amount}%")

    def run(self):
        """メインループの実行"""
        self.initialize()

        #for loop in range(self.loop_amount):
        for loop in range(5):
            print(f"{loop}ステップ目")
            # 各ループの開始時に全UAVペア間のノイズ付き測定値を事前計算してキャッシュ
            measurements_cache = self.build_measurements_cache()

            # 1.直接推定の実行
            self.exec_direct_estimation(measurements_cache, loop)

            # 2.融合推定の実行
            self.exec_fused_estimation(measurements_cache, loop)

            # 3.制御入力の算出と適用
            self.apply_control_input(measurements_cache, loop)

            # 結果をlogに保存する（update_state前の位置を記録）
            self.data_logger.logging_timestamp(loop * self.dt)

            # 全UAVの軌道を記録（現在の位置 k を記録）
            for uav in self.uavs:
                self.data_logger.logging_uav_trajectories(uav_id=uav.id, uav_position=uav.true_position.copy())

            # 全UAVの真の状態を k+1 に更新
            for uav in self.uavs:
                uav.update_state(dt=self.dt)

            for uav in self.uavs:
                if uav.id == self.params['TARGET_ID']:
                    continue
                # k+1時点での推定誤差を計算
                error_distance = self.calc_RL_estimation_error(uav.id, self.params['TARGET_ID'], loop+1)
                # 推定誤差をロギング
                self.data_logger.logging_fused_RL_error(uav_id=uav.id, error=error_distance)

            self.show_simulation_progress(loop=loop)

        # ロギングした推定誤差をcsv出力
        trajectory_filename = self.data_logger.save_UAV_trajectories_data_to_csv()
        error_filename = self.data_logger.save_fused_RL_errors_to_csv()

        # グラフ生成
        Plotter.plot_UAV_trajectories_from_csv(trajectory_filename)
        Plotter.plot_fused_RL_errors_from_csv(error_filename)

        # 統計情報の表示と保存
        self.data_logger.print_fused_RL_error_statistics(transient_time=120.0)
        self.data_logger.save_fused_RL_error_statistics(transient_time=120.0)
        self.data_logger.save_fused_RL_error_statistics(transient_time=120.0, format='txt')

if __name__ == '__main__':
    # 設定ファイルから読み込む
    # JSON形式
    #simulation_params = ConfigLoader.load('../config/simulation_config.json')
    from sensor_sim_mock import MockSensor
    from sensor_sim_coppelia import CoppeliaSensor
    # YAML形式
    simulation_params = ConfigLoader.load('../config/simulation_config.yaml')

    controller = MainController(simulation_params, sensor=MockSensor())
    controller.run()
