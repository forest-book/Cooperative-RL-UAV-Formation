# Cooperative-RL-UAV-Formation

協調型相対自己位置（Relative Localization, RL）推定に基づくUAVフォーメーション制御のシミュレーションフレームワーク

## 概要

本プロジェクトは、複数のUAV（無人航空機）が相互に相対位置を推定しながら、所望のフォーメーションを維持して飛行する協調制御システムのシミュレーション実装です。

### 主な特徴

- **協調型RL推定アルゴリズム**: 直接推定と間接推定を融合した相対位置推定
- **分散型フォーメーション制御**: 各UAVが隣接機との情報交換のみで制御を実現
- **ノイズロバスト設計**: センサノイズ（速度・距離・距離変化率）を考慮した推定
- **柔軟な設定システム**: JSON/YAML形式での実験条件設定
- **可視化・分析機能**: 軌跡、推定誤差、機体間距離などの自動グラフ生成

## システム構成

### アーキテクチャ

```
Cooperative-RL-UAV-Formation/
├── config/                    # 設定ファイル
│   ├── simulation_config.json # JSON形式の設定
│   └── simulation_config.yaml # YAML形式の設定
├── main/                      # メインソースコード
│   ├── main.py               # メインコントローラ
│   ├── quadcopter.py         # UAVクラス
│   ├── estimator.py          # RL推定アルゴリズム
│   ├── control_input.py      # フォーメーション制御
│   ├── data_logger.py        # データロギング
│   ├── plotter.py            # グラフ生成
│   ├── config_loader.py      # 設定ファイル読み込み
│   ├── interface_sensor.py   # センサインターフェース
│   ├── sensor_sim_mock.py    # モックセンサ実装
│   └── sensor_sim_coppelia.py # CoppeliaSim連携用
├── sandbox/                   # 実験・テスト用スクリプト
└── data/                      # 出力データ（CSV、グラフ、統計）
```

## インストール

### 必須要件

- Python 3.8以上
- NumPy
- Pandas
- Matplotlib

### インストール手順

```bash
# リポジトリのクローン（<GITHUB_USER_OR_ORG> を実際のユーザー名または組織名に置き換えてください）
git clone https://github.com/<GITHUB_USER_OR_ORG>/Cooperative-RL-UAV-Formation.git
cd Cooperative-RL-UAV-Formation

# 仮想環境の作成（推奨）
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# 依存パッケージのインストール
pip install numpy pandas matplotlib

# YAMLサポート（オプション）
pip install pyyaml
```

## 使い方

### 基本的な実行

```bash
cd main
python main.py
```

デフォルトでは `config/simulation_config.yaml` の設定が使用されます。

### 設定のカスタマイズ

#### 1. 既存の設定ファイルを編集

```yaml
# config/simulation_config.yaml
DURATION: 300      # シミュレーション時間 [秒]
T: 0.02           # サンプリング周期 [秒]
GAMMA: 0.06       # 推定ゲイン

# UAV初期位置
INITIAL_POSITIONS:
  1: [-20, -20]
  2: [ 10, -10]
  3: [ 20,  20]

# 隣接関係（通信トポロジー）
NEIGHBORS:
  1: [2, 3]
  2: [1, 3]
  3: [1, 2]

# 制御パラメータ
GAMMA1: 1.0       # 速度合意ゲイン
GAMMA2: 0.00014   # フォーメーション制御ゲイン
DIST: 15          # 目標相対距離 [m]

# ノイズパラメータ
NOISE:
  delta_bar: 0.5        # 速度ノイズ幅 [m/s]
  dist_bound: 0.05      # 距離ノイズ幅 [m]
  initialization_bound: 0  # 初期推定ノイズ幅 [m]
```

#### 2. カスタム設定ファイルの使用

```python
# main.py内で設定ファイルのパスを変更
simulation_params = ConfigLoader.load('../config/my_experiment.yaml')
```

### 出力データ

シミュレーション実行後、以下のファイルが生成されます：

```
data/
├── csv/
│   ├── trajectories/          # UAV軌跡データ
│   ├── RL_errors/            # 推定誤差データ
│   ├── inter_uav_dist/       # 機体間距離データ
│   └── relative_velocity/    # 相対速度データ
├── graph/
│   ├── trajectories/          # 軌跡グラフ（PNG）
│   ├── RL_errors/            # 推定誤差グラフ
│   ├── inter_uav_dist/       # 距離グラフ
│   └── relative_velocity/    # 相対速度グラフ
└── statistics/
    ├── json/                  # 統計情報（JSON形式）
    └── txt/                   # 統計情報（テキスト形式）
```

## 理論的背景

### 直接RL推定（式(1)）

隣接UAV間の相対位置を直接観測値から推定：

```
χ̂_{ij,i}(k+1) = χ̂_{ij,i}(k) + T·v_{ij}(k) + γT·v_{ij}(k)·[d_{ij}(k)·ḋ_{ij}(k) - v_{ij}^T(k)·χ̂_{ij,i}(k)]
```

### 融合RL推定（式(5)）

直接推定と隣接機からの間接推定を融合：

```
π_{ij,i}(k+1) = π_{ij,i}(k) + T·v_{ij}(k) + κ^D·[χ̂_{ij,i}(k) - π_{ij,i}(k)] + Σ κ^I·[χ̂_{ij,r}(k) - π_{ij,i}(k)]
```

### フォーメーション制御（式(9)）

融合推定値を用いた分散型制御入力：

```
v_i(k+1) = v_i(k) + γ₁T·Σ v_{ij}^i(k) + γ₂T·Σ [(d_{ij}(k))² - (d_{ij}*)²]·π_{ij,i}(k)
```

## アルゴリズムの流れ

1. **初期化**
   - UAVの初期位置・速度設定
   - 隣接関係（通信トポロジー）の定義
   - 推定値の初期化（真値 + ノイズ）

2. **メインループ（各時刻k）**
   - センサ測定値の取得（ノイズ付き）
   - 直接RL推定の実行
   - 融合RL推定の実行
   - 制御入力の計算
   - UAV状態の更新
   - データロギング

3. **結果出力**
   - CSV形式でのデータ保存
   - グラフの自動生成
   - 統計情報の計算・保存

## 主要クラスの説明

### `MainController`
- シミュレーション全体の統括管理
- 初期化、メインループ、結果出力を制御

### `UAV`
- 各UAVの状態管理（位置、速度、推定値）
- ダイナミクスの更新

### `Estimator`
- 直接RL推定の計算
- 融合RL推定の計算
- 推定重み（κ）の算出

### `ControlInput`
- フォーメーション制御入力の計算
- 速度合意項とフォーメーション制御項の統合

### `DataLogger`
- 時系列データの収集
- CSV出力
- 統計情報の計算

### `Plotter`
- CSVファイルからのグラフ生成
- 学術論文スタイルの可視化

## センサモデル

### MockSensor（デフォルト）
Pythonのみでのシミュレーション用モックセンサ：
- 相対速度：ガウス分布ノイズ N(0, σ²)
- 距離：UWBモジュールを模擬した測距
- 距離変化率：幾何学的計算 + ノイズ

### CoppeliaSensor（実装予定）
CoppeliaSim物理シミュレータとの連携用

## カスタマイズ

### 新しいセンサの追加

```python
from interface_sensor import ISensor

class MySensor(ISensor):
    def get_velocity_info(self, uav_i, uav_j, delta_bar, *, add_vel_noise=False):
        # 実装
        pass

    def get_distance_info(self, uav_i, uav_j, dist_bound, *, add_dist_noise=False):
        # 実装
        pass

    def get_distance_rate_info(self, uav_i, uav_j, dist_bound, *, add_dist_rate_noise=False):
        # 実装
        pass

# main.pyで使用
controller = MainController(simulation_params, sensor=MySensor())
```

### 制御則の変更

`control_input.py`の`calc_RL_based_control_input`メソッドを編集して、独自の制御アルゴリズムを実装できます。

## 実験例

### 3機フォーメーション（三角形）

```yaml
INITIAL_POSITIONS:
  1: [-20, -20]
  2: [ 10, -10]
  3: [ 20,  20]

NEIGHBORS:
  1: [2, 3]
  2: [1, 3]
  3: [1, 2]

DIST: 15  # 15m間隔の正三角形
```

### 6機フォーメーション（リーダー・フォロワー）

```yaml
INITIAL_POSITIONS:
  1: [0, 0]      # リーダー
  2: [2, -30]
  3: [20, -15]
  4: [-20, 8]
  5: [-14, 8]
  6: [-10, -30]

NEIGHBORS:
  1: []          # リーダーは隣接機なし
  2: [1]
  3: [1, 4, 5]
  4: [1]
  5: [3, 4]
  6: [4]
```

## パフォーマンス指標

シミュレーション終了後、以下の統計が自動計算されます：

- **推定誤差の平均・分散・標準偏差**（UAVペアごと）
- **過渡状態除外**：デフォルトで最初の10秒を除外
- **JSON/TXT形式での保存**

例：
```
====================================================================
  融合RL推定誤差の統計 (10.0秒後から安定状態)
====================================================================
UAV Pair   | Mean Error (m)     | Variance       | Std Dev (m)
--------------------------------------------------------------------
 1-2       | 0.123456           | 0.001234       | 0.035123
 1-3       | 0.098765           | 0.000987       | 0.031415
 2-3       | 0.156789           | 0.002456       | 0.049558
====================================================================
```

## トラブルシューティング

### よくある問題

1. **`pyyaml`が見つからない**
   ```bash
   pip install pyyaml
   ```

2. **グラフが表示されない**
   - バックエンドの設定を確認：`matplotlib.use('TkAgg')`
   - または画像ファイルとして保存されているか確認

3. **推定が発散する**
   - `GAMMA`（推定ゲイン）を小さくする
   - `GAMMA2`（制御ゲイン）を調整する
   - ノイズパラメータを小さくする

## 開発ガイドライン

### コーディング規約
- `.editorconfig`に準拠
- Pythonファイル：最大行長88文字
- インデント：スペース4つ

### プルリクエスト
`.github/pull_request_template.md`のテンプレートを使用してください。

## ライセンス

（ライセンス情報を追加してください）

## 引用

本実装を研究で使用する場合は、以下の論文を引用してください：

（関連論文情報を追加してください）

## 貢献

バグ報告、機能提案、プルリクエストを歓迎します。

## 連絡先

本プロジェクトに関するお問い合わせや議論は、本リポジトリの GitHub Issues または（有効な場合）Discussions を通じて行ってください。

---

**更新日**: 2026年1月3日
