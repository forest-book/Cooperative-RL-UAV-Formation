import numpy as np
import matplotlib.pyplot as plt


@staticmethod
def calc_adaptive_gamma2_sigmoid(
    dist_error_sq: float,
    gamma2_min: float,
    gamma2_max: float,
    steepness: float = 0.05,      # 遷移の急峻さ
    error_center: float = 100.0   # 遷移の中心点（d²単位）
) -> float:
    """
    シグモイド関数で距離誤差をgamma2に写像
    
    誤差が大きい → gamma2_min（抑制）
    誤差が小さい → gamma2_max（加速）
    """
    abs_error = abs(dist_error_sq)
    
    # シグモイド: 誤差が小さいほど1に近づく
    sigmoid = 1.0 / (1.0 + np.exp(-steepness * (error_center - abs_error)))
    
    # [gamma2_min, gamma2_max] に線形写像
    gamma2 = gamma2_min + (gamma2_max - gamma2_min) * sigmoid
    
    return gamma2

def plot_gamma2_behavior():
    # --- パラメータ設定 ---
    gamma2_min = 0.000014
    gamma2_max = 0.014
    error_center = 100.0
    
    # 評価する誤差の範囲 (0 から 200まで)
    x_errors = np.linspace(0, 400, 500)

    # 異なる steepness (急峻さ) での挙動を比較
    steepness_list = [0.05, 0.1, 0.5]
    colors = ['blue', 'green', 'orange']

    plt.figure(figsize=(10, 6))

    # --- プロット描画 ---
    for stp, col in zip(steepness_list, colors):
        y_gamma = calc_adaptive_gamma2_sigmoid(
            x_errors, gamma2_min, gamma2_max, steepness=stp, error_center=error_center
        )
        label = f'Steepness = {stp}'
        if stp == 0.05: label += ' (Default)'
        
        plt.plot(x_errors, y_gamma, label=label, color=col, linewidth=2)

    # --- グラフの装飾 ---
    
    # 境界線の描画
    plt.axvline(x=error_center, color='red', linestyle='--', alpha=0.6, label=f'Center ({error_center})')
    plt.axhline(y=gamma2_max, color='gray', linestyle=':', alpha=0.5)
    plt.axhline(y=gamma2_min, color='gray', linestyle=':', alpha=0.5)

    # エリアの意味を注釈で追記
    plt.text(10, gamma2_max - 0.2, "誤差小 → 加速 (Max)", fontsize=10, fontweight='bold', color='#333')
    plt.text(150, gamma2_min + 0.2, "誤差大 → 抑制 (Min)", fontsize=10, fontweight='bold', color='#333')

    # タイトルとラベル
    plt.title("Adaptive Gamma2 Sigmoid Transition", fontsize=14)
    plt.xlabel("Squared Distance Error ($d^2$)", fontsize=12)
    plt.ylabel("Gamma2 Value", fontsize=12)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Y軸の範囲を少し広めに
    plt.ylim(gamma2_min - 0.5, gamma2_max + 0.5)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_gamma2_behavior()