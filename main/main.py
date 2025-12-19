import numpy as np
from typing import List, Tuple

from quadcopter import UAV
from estimator import Estimator
from data_logger import DataLogger
from plotter import Plotter
from config_loader import ConfigLoader

class MainController:
    """アプリケーション全体を管理し，メインループを実行する"""
    def __init__(self, params: dict):
        self.params = params
        self.uavs: List[UAV] = []
        self.loop_amount: int = 0
        self.dt = params['T']   # サンプリング周期
        self.event = params['EVENT']    # t=100sで外乱を加えるか否か

        self.estimator = Estimator()
        self.data_logger = DataLogger()

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
        target_id = self.params['TARGET_ID']
        target_uav = self.get_uav_by_id(target_id)
        for uav_i in self.uavs:
            if uav_i.id == target_id:
                continue  # TARGET自身は自分への推定を行わない
            true_initial_rel_pos: np.ndarray = target_uav.true_position - uav_i.true_position
            # 一様乱数を生成して真値に加算
            noise = np.random.uniform(-noise_bound, noise_bound, size=true_initial_rel_pos.shape)
            noisy_initial_rel_pos = true_initial_rel_pos + noise
            key = self.make_fused_estimate_key(uav_i.id, target_id)
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

        for uav in self.uavs:
            print(f"uav_{uav.id}")
            print(uav.id)
            print(uav.neighbors)
            print(uav.true_position)
            print(uav.true_velocity)

        # k=0での直接推定値を設定(直接推定値の初期化)
        # 隣接機に対してのみ初期化
        self.initialize_direct_estimates()

        for uav in self.uavs:
            print(f"uav_{uav.id}")
            print(uav.direct_estimates)
        return

        # k=0での融合推定値を設定(融合推定値の初期化)
        # UAV_i(i=2~6)から見たUAV1の相対位置を融合推定
        self.initialize_fused_estimates()

        # 推定式はステップk(自然数)毎に状態を更新するため
        self.loop_amount = int(self.params['DURATION'] / self.params['T'])

    def get_noisy_measurements(self, uav_i: UAV, uav_j: UAV, *, add_vel_noise=True, add_dist_noise=True, add_dist_rate_noise=True) -> Tuple[np.ndarray, float, float]:
        """
        2UAV間の真の状態に基づき、ノイズが付加された測定値を生成する
        シミュレータ上に測距モジュールがあるなら不要となる関数
        
        ノイズモデル:
        - ガウス分布（正規分布）に基づくノイズを使用
        - 一様分布の±bound/2の範囲を、ガウス分布の3σに相当すると解釈
        - 99.7%の値が±3σ以内に収まり、時折真値に近い測定も得られる
        """
        # 真の相対値
        true_x_ij = uav_j.true_position - uav_i.true_position
        true_v_ij = uav_j.true_velocity - uav_i.true_velocity # 自機の速度情報はUWB RCM通信で送られてくる
        true_d_ij = np.linalg.norm(true_x_ij) # UWBモジュールでの測距を模している
        # 論文式(1)の上あたりの方程式から算出される
        true_d_dot_ij = (true_x_ij @ true_v_ij) / (true_d_ij + 1e-9) # ゼロ除算防止

        # ノイズモデル (4.1節)
        delta_bar = self.params['NOISE']['delta_bar']
        dist_bound = self.params['NOISE']['dist_bound']

        # 速度ノイズ: ガウス分布 N(0, σ²)
        # 元の一様分布の全幅δ̄を±3σ（6σ）に対応させる → σ = δ̄/6
        sigma_v = delta_bar / 6.0
        vel_noise = np.random.normal(0, sigma_v, size=2) if add_vel_noise else np.zeros(2)
        
        # 距離ノイズ: ガウス分布 N(0, σ²)
        # 元の一様分布の全幅boundを±3σ（6σ）に対応させる → σ = bound/6
        sigma_d = dist_bound / 6.0
        dist_noise = np.random.normal(0, sigma_d) if add_dist_noise else 0.0
        
        # 距離変化率ノイズ: ガウス分布 N(0, σ²)
        # 距離ノイズと同じ標準偏差σ_dを使用
        dist_rate_noise = np.random.normal(0, sigma_d) if add_dist_rate_noise else 0.0

        return true_v_ij + vel_noise, true_d_ij + dist_noise, true_d_dot_ij + dist_rate_noise

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
    
    def show_simulation_progress(self, loop):
        if(loop * 100 // self.loop_amount) > ((loop - 1) *100 // self.loop_amount):
            print(f"simulation progress: {loop *100 // self.loop_amount}%")

    def run(self):
        """メインループの実行"""
        self.initialize()
        return

        for loop in range(self.loop_amount):
            # 各ループの開始時に全UAVペア間のノイズ付き測定値を事前計算してキャッシュ
            measurements_cache = {}
            for uav_i in self.uavs:
                for uav_j in self.uavs:
                    if uav_i.id == uav_j.id:
                        continue
                    # 測定は方向性があるため、キー (i, j) は「uav_i から uav_j への測定」を表す（順序は正規化しない）
                    key = (uav_i.id, uav_j.id)
                    noisy_v, noisy_d, noisy_d_dot = self.get_noisy_measurements(
                        uav_i, uav_j, 
                        add_vel_noise=True, 
                        add_dist_noise=True, 
                        add_dist_rate_noise=True
                    )
                    measurements_cache[key] = (noisy_v, noisy_d, noisy_d_dot)

            # 1.直接推定の実行
            for uav_i in self.uavs:
                for neighbor_id in uav_i.neighbors:
                    
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
                    # keyは157行目で生成済みなので再利用
                    uav_i.direct_estimates[key].append(next_direct.copy())

            # 2.融合推定の実行
            # UAV_i(i=2~6)がUAV_1への融合推定値を算出する
            target_j_id: int = self.params['TARGET_ID']
            for uav_i in self.uavs:
                if uav_i.id == target_j_id:
                    continue # UAV1 (j=1) は自身への推定を行わない
                
                # 重みκを計算
                kappa_D, kappa_I = self.estimator.calc_estimation_kappa(uav_i.neighbors.copy(), target_j_id) # Listは参照渡しなのでcopyを渡す

                # キャッシュからノイズ付き相対速度 v_ij を取得
                noisy_v_ij, _, _ = measurements_cache[(uav_i.id, target_j_id)]

                # 直接推定値と融合推定値を持ってくる
                direct_key = self.make_direct_estimate_key(uav_i.id, target_j_id)
                fused_key = self.make_fused_estimate_key(uav_i.id, target_j_id)
                chi_hat_ij_i_k = uav_i.direct_estimates[direct_key] # k=loopの時の直接推定値を持ってくる
                pi_ij_i_k = uav_i.fused_estimates[fused_key]

                # 間接推定値のリストを作成
                indirect_estimates_list: List = []
                for r_id in uav_i.neighbors:
                    if r_id == target_j_id: # r(間接機)はtarget(推定対象)であってはならない
                        continue

                    uav_r = self.get_uav_by_id(r_id) #uav_iの隣接機UAVオブジェクト
                    
                    # uav_i(自機)からuav_r(間接機)への直接推定値
                    direct_key_ir = self.make_direct_estimate_key(uav_i.id, uav_r.id)
                    chi_hat_ir_i_k = uav_i.direct_estimates[direct_key_ir]
                    # uav_r(間接機)からtarget(推定対象)への融合推定値
                    fused_key_rj = self.make_fused_estimate_key(uav_r.id, target_j_id)
                    pi_rj_r_k = uav_r.fused_estimates[fused_key_rj]
                    # uav_i(自機)からtarget(推定対象)への間接推定値
                    chi_hat_ij_r_k: np.ndarray = chi_hat_ir_i_k[loop] + pi_rj_r_k[loop]
                    # リストに格納
                    indirect_estimates_list.append(chi_hat_ij_r_k.copy())
                
                next_fused = self.estimator.calc_fused_RL_estimate(
                    pi_ij_i_k=pi_ij_i_k[loop],
                    direct_estimate_x_hat=chi_hat_ij_i_k[loop] if kappa_D!=0 else np.zeros(2),
                    indirect_estimates=indirect_estimates_list,
                    noisy_v=noisy_v_ij,
                    T=self.dt,
                    kappa_D=kappa_D,
                    kappa_I=kappa_I
                ) # 次のステップ(k=loop + 1)の時の相対位置を融合推定
                
                # fused_keyは188行目で生成済みなので再利用
                uav_i.fused_estimates[fused_key].append(next_fused.copy())

            # 結果をlogに保存する（update_state前の位置を記録）
            self.data_logger.logging_timestamp(loop * self.dt)

            # 全UAVの軌道を記録（現在の位置 k を記録）
            for uav in self.uavs:
                self.data_logger.logging_uav_trajectories(uav_id=uav.id, uav_position=uav.true_position.copy())

            # 全UAVの真の状態を k+1 に更新
            for uav in self.uavs:
                uav.update_state(t=loop+1, dt=self.dt, event=self.params['EVENT'])

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
    
    # YAML形式
    simulation_params = ConfigLoader.load('../config/simulation_config.yaml')

    controller = MainController(simulation_params)
    controller.run()
