import pandas as pd
import matplotlib.pyplot as plt
import os

def set_academic_style():
    """論文掲載用のグラフスタイル設定 (IEEE風)"""
    # フォント設定 (Times New Roman推奨、なければSerif)
    plt.rcParams['font.family'] = 'serif'
    try:
        plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
    except:
        pass
    plt.rcParams['mathtext.fontset'] = 'stix'  # 数式フォント

    # フォントサイズ
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.labelsize'] = 14
    plt.rcParams['legend.fontsize'] = 12

    # 目盛り設定 (内向き)
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['ytick.direction'] = 'in'
    plt.rcParams['xtick.major.size'] = 5
    plt.rcParams['ytick.major.size'] = 5

    # 線の太さなど
    plt.rcParams['lines.linewidth'] = 2
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.6
    plt.rcParams['figure.dpi'] = 150

def plot_distance_metrics(csv_path: str):
    """論文 Fig. 7(c) の距離推移グラフを描画"""
    if not os.path.exists(csv_path):
        print(f"Error: File not found -> {csv_path}")
        return

    try:
        data = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    set_academic_style()

    # グラフ描画
    plt.figure(figsize=(8, 5))

    # 論文Fig 7の配色に近い色 (緑, 赤, 青/シアン)
    colors = ['#2CA02C', '#D62728', '#1F77B4']

    # 'dist_' で始まるカラムを抽出してプロット
    dist_cols = [c for c in data.columns if c.startswith('dist_')]

    for idx, col in enumerate(dist_cols):
        # 凡例ラベルの作成 (例: dist_1-2 -> d^{01})
        # ※実装ID(1,2,3)を論文ID(0,1,2)に変換して表示
        try:
            pair_str = col.replace('dist_', '')
            u_a, u_b = map(int, pair_str.split('-'))
            label = f'$d^{{{u_a-1}{u_b-1}}}$'
        except:
            label = col # 変換失敗時はそのまま

        # データ補間（滑らかに表示したい場合）
        # time = data['time']
        # values = data[col]
        # plt.plot(time, values, label=label, color=colors[idx % 3])

        # マーカーなしの実線
        plt.plot(data['time'], data[col], label=label, color=colors[idx % 3])

    # 目標距離のガイドライン (15m, 30m)
    plt.axhline(y=15, color='gray', linestyle=':', linewidth=1.5, alpha=0.8) # Target 15m
    plt.axhline(y=30, color='gray', linestyle=':', linewidth=1.5, alpha=0.8) # Target 30m

    # テキスト注釈 (論文のように "15m", "30m" と入れても良い)
    plt.text(0.5, 15.5, 'Target: 15m', fontsize=10, color='gray')
    plt.text(0.5, 30.5, 'Target: 30m', fontsize=10, color='gray')

    # タイトル・軸ラベル
    plt.title('Inter-UAV Distance')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')

    # 軸範囲 (論文に合わせて適宜調整)
    plt.ylim(0, 40)
    plt.xlim(0, 40) # サンプルデータに合わせて調整

    # 凡例
    plt.legend(loc='lower right', ncol=3)

    plt.tight_layout()

    # 保存
    output_filename = "fig7c_reproduction.png"
    plt.savefig(output_filename)
    print(f"Graph saved as {output_filename}")
    plt.show()

if __name__ == "__main__":
    target_csv = "sample_metrics_distance.csv"
    plot_distance_metrics(target_csv)
