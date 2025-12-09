import pandas as pd
import matplotlib.pyplot as plt
import os

def set_academic_style():
    """論文掲載用のグラフスタイル設定"""
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
    plt.rcParams['mathtext.fontset'] = 'stix'
    
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.labelsize'] = 14
    plt.rcParams['legend.fontsize'] = 10
    
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['ytick.direction'] = 'in'
    
    plt.rcParams['lines.linewidth'] = 1.5
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['grid.alpha'] = 0.6
    plt.rcParams['figure.dpi'] = 150

def plot_trajectory_graph(csv_path: str):
    if not os.path.exists(csv_path):
        print(f"Error: File not found -> {csv_path}")
        return

    # 1. データ読み込み
    try:
        data = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    # 2. スタイル適用
    set_academic_style()

    # 3. プロット作成
    # 論文のFig 7は正方形に近いアスペクト比
    plt.figure(figsize=(6, 6))
    
    # 論文Fig 7の色に近い設定 (Red, Blue, Cyan/Purple)
    # ※論文内のUAV ID (0,1,2) と実装ID (1,2,3) の対応を意識
    # ここでは便宜上、CSVのカラム順(uav1, uav2, uav3)に色を割り当てます
    uav_colors = {
        'uav1': '#D62728',  # Red
        'uav2': '#1F77B4',  # Blue
        'uav3': '#17BECF'   # Cyan
    }
    
    # CSVのカラム名からUAVを抽出 ('uavX_true_pos_x' の形式を想定)
    uav_ids = set()
    for col in data.columns:
        if 'true_pos_x' in col:
            uav_ids.add(col.split('_')[0])
    uav_ids = sorted(list(uav_ids)) # ['uav1', 'uav2', 'uav3']

    for u_id in uav_ids:
        x_col = f'{u_id}_true_pos_x'
        y_col = f'{u_id}_true_pos_y'
        
        if x_col in data.columns and y_col in data.columns:
            x = data[x_col]
            y = data[y_col]
            color = uav_colors.get(u_id, 'black')
            
            # 軌跡のプロット
            # 論文に合わせてラベルを調整 (例: uav1 -> UAV0)
            # ここではシンプルにそのまま表示します
            label_name = u_id.upper() 
            plt.plot(x, y, label=label_name, color=color)
            
            # 始点マーカー (白抜きの丸)
            plt.scatter(x.iloc[0], y.iloc[0], marker='o', color=color, s=40, facecolors='none', edgecolors=color, zorder=5)
            
            # 終点マーカー (塗りつぶしの菱形)
            plt.scatter(x.iloc[-1], y.iloc[-1], marker='D', color=color, s=40, zorder=5)
            
            # 終点付近にテキスト注釈を追加
            plt.text(x.iloc[-1] + 3, y.iloc[-1], label_name, fontsize=9, color=color, va='center')

    # 4. タイトル・軸ラベル設定 (論文Fig 7準拠)
    plt.title('Trajectories of 3 UAVs')
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    
    # 凡例
    plt.legend(loc='upper right')
    
    # アスペクト比を1:1に固定 (軌跡の歪みを防ぐため重要)
    plt.axis('equal')
    plt.grid(True)
    plt.tight_layout()
    
    # 5. 保存と表示
    output_filename = "trajectory_graph.png"
    plt.savefig(output_filename)
    print(f"Graph saved as {output_filename}")
    plt.show()

if __name__ == "__main__":
    target_csv = "sample_trajectories.csv"
    target_csv_alt = "sample_trajectories_curved.csv"
    plot_trajectory_graph(target_csv_alt)
