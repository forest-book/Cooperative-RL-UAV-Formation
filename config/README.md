# 設定ファイルディレクトリ

このディレクトリにはシミュレーションの設定ファイルが格納されています。

## ファイル形式

### JSON形式 (simulation_config.json)

- 標準的なJSON形式
- Pythonの標準ライブラリのみで読み込み可能
- 追加のパッケージインストール不要

### YAML形式 (simulation_config.yaml)

- 人間が読みやすい形式
- コメントが書ける
- `pyyaml`パッケージが必要: `pip install pyyaml`

## パラメータ説明

### 基本設定

- **DURATION**: シミュレーション時間 [秒]
- **T**: サンプリング周期 [秒]
- **GAMMA**: 推定ゲイン γ
- **TARGET_ID**: 推定目標UAVのID (通常は1)

### シナリオ設定

- **EVENT**: 飛行シナリオ
  - `CONTINUOUS`: 通常飛行
  - `SUDDEN_TURN`: 急旋回

### UAV配置

- **INITIAL_POSITIONS**: 各UAVの初期位置 [x, y] (m)
- **NEIGHBORS**: 各UAVの隣接機リスト

### ノイズ設定

- **delta_bar**: 速度ノイズの幅 [m/s]
- **dist_bound**: 距離ノイズの幅 [m]

## 使用方法

```python
from config_loader import ConfigLoader

# JSON形式で読み込み
params = ConfigLoader.load('../config/simulation_config.json')

# YAML形式で読み込み (pyyamlが必要)
params = ConfigLoader.load('../config/simulation_config.yaml')

# 自動判定で読み込み
params = ConfigLoader.load('../config/simulation_config.json')  # 拡張子で自動判定
```

## カスタム設定の作成

既存の設定ファイルをコピーして編集することで、異なる実験条件を簡単に作成できます：

```bash
cp simulation_config.json my_experiment.json
# my_experiment.json を編集
```

```python
# カスタム設定で実行
params = ConfigLoader.load('../config/my_experiment.json')
```
