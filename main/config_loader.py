"""
設定ファイルローダー
JSON形式またはYAML形式の設定ファイルを読み込む
"""
import json
import os
from typing import Dict, Any
from quadcopter import Scenario


class ConfigLoader:
    """設定ファイルを読み込むクラス"""

    @staticmethod
    def load_from_json(filepath: str) -> Dict[str, Any]:
        """
        JSON形式の設定ファイルを読み込む

        Args:
            filepath (str): 設定ファイルのパス

        Returns:
            Dict[str, Any]: 設定パラメータの辞書
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"設定ファイルが見つかりません: {filepath}")

        with open(filepath, 'r', encoding='utf-8') as f:
            config = json.load(f)

        # JSONのキーは文字列なので整数に変換
        config['INITIAL_POSITIONS'] = {
            int(key): value for key, value in config['INITIAL_POSITIONS'].items()
        }
        config['NEIGHBORS'] = {
            int(key): value for key, value in config['NEIGHBORS'].items()
        }

        # EVENTを文字列からScenarioオブジェクトに変換
        config['EVENT'] = ConfigLoader._parse_scenario(config['EVENT'])

        return config

    @staticmethod
    def load_from_yaml(filepath: str) -> Dict[str, Any]:
        """
        YAML形式の設定ファイルを読み込む

        Args:
            filepath (str): 設定ファイルのパス

        Returns:
            Dict[str, Any]: 設定パラメータの辞書
        """
        try:
            import yaml
        except ImportError:
            raise ImportError(
                "YAMLファイルを読み込むにはpyyamlが必要です。\n"
                "インストール: pip install pyyaml"
            )

        if not os.path.exists(filepath):
            raise FileNotFoundError(f"設定ファイルが見つかりません: {filepath}")

        with open(filepath, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)

        # EVENTを文字列からScenarioオブジェクトに変換
        config['EVENT'] = ConfigLoader._parse_scenario(config['EVENT'])

        return config

    @staticmethod
    def _parse_scenario(scenario_str: str) -> Scenario:
        """
        シナリオ名文字列をScenarioオブジェクトに変換

        Args:
            scenario_str (str): シナリオ名 ('CONTINUOUS', 'SUDDEN_TURN')

        Returns:
            Scenario: Scenarioオブジェクト
        """
        scenario_map = {
            'CONTINUOUS': Scenario.CONTINUOUS,
            'SUDDEN_TURN': Scenario.SUDDEN_TURN,
        }

        scenario_str_upper = scenario_str.upper()
        if scenario_str_upper not in scenario_map:
            raise ValueError(
                f"不明なシナリオ: {scenario_str}\n"
                f"利用可能なシナリオ: {list(scenario_map.keys())}"
            )

        return scenario_map[scenario_str_upper]

    @staticmethod
    def load(filepath: str) -> Dict[str, Any]:
        """
        ファイル拡張子から自動判定して設定を読み込む

        Args:
            filepath (str): 設定ファイルのパス

        Returns:
            Dict[str, Any]: 設定パラメータの辞書
        """
        ext = os.path.splitext(filepath)[1].lower()

        if ext == '.json':
            return ConfigLoader.load_from_json(filepath)
        elif ext in ['.yaml', '.yml']:
            return ConfigLoader.load_from_yaml(filepath)
        else:
            raise ValueError(
                f"サポートされていないファイル形式: {ext}\n"
                f"利用可能な形式: .json, .yaml, .yml"
            )
