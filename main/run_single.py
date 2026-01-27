import sys
import datetime

from config_loader import ConfigLoader
from controller import MainController
from sensor_sim_mock import MockSensor
from sensor_sim_coppelia import CoppeliaSensor

if __name__ == '__main__':
    # 設定ファイルから読み込む
        # YAML形式
        simulation_params = ConfigLoader.load('../config/config_dist_change.yaml')
        #simulation_params = ConfigLoader.load('../config/config_uav5_v2.yaml')

        controller = MainController(simulation_params, sensor=MockSensor())
        controller.run()
