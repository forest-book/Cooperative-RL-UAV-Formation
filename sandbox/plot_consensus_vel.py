import pandas as pd
import matplotlib.pyplot as plt
import os

def set_academic_style():
    """論文掲載用のグラフスタイル設定 (IEEE風)"""
    # フォント設定 (Times New Roman推奨)
    plt.rcParams['font.family'] = 'serif'
    try:
        plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
    except Exception:
        pass
    plt.rcParams['mathtext.fontset'] = 'stix'
    
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
    plt.rcParams['lines.linewidth'] = 1.5
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.6
    plt.rcParams['figure.dpi'] = 150

def plot_velocity_metrics_combined(csv_path: str):
    """
    論文 Fig. 7(d) の速度合意グラフを描画 (統合CSV対応版)
    
    Args:
        csv_path (str): 'velocity_consensus_rl' と 'velocity_consensus_nl' カラムを含むCSVパス
    """
    if not os.path.exists(csv_path):
        print(f"Error: File not found -> {csv_path}")
        return

    try:
        data = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    set_academic_style()

    plt.figure(figsize=(8, 5))
    
    # 1. Noise + RL estimates (青色実線)
    if 'velocity_consensus_rl' in data.columns:
        plt.plot(data['time'], data['velocity_consensus_rl'], 
                 color='blue', linewidth=1.5, label='Noise+RL estimates')
    else:
        print("Warning: Column 'velocity_consensus_rl' not found.")

    # 2. Noiseless (赤色点線)
    if 'velocity_consensus_nl' in data.columns:
        plt.plot(data['time'], data['velocity_consensus_nl'], 
                 color='red', linewidth=1.5, linestyle='--', label='Noiseless')
    else:
        print("Warning: Column 'velocity_consensus_nl' not found.")

    # タイトル・軸ラベル (数式を含む)
    plt.title(r'Average relative velocity: $\frac{1}{N}\sum ||v^{ij}||$')
    plt.xlabel('Time (s)')
    plt.ylabel('Relative velocity (m/s)')
    
    # 軸範囲 (論文に合わせて調整)
    plt.ylim(0, 9)
    if 'time' in data.columns:
        max_time = data['time'].max()
        plt.xlim(0, max_time)
    
    # 凡例
    plt.legend(loc='upper right')
    
    plt.tight_layout()
    
    # 保存
    output_filename = "fig7d_combined.png"
    plt.savefig(output_filename)
    print(f"Graph saved as {output_filename}")
    plt.show()

if __name__ == "__main__":
    # 統合されたCSVファイルを指定
    target_csv = "sample_metrics_velocity_combined.csv"
    
    plot_velocity_metrics_combined(target_csv)