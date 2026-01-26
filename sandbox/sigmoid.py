import numpy as np
import matplotlib.pyplot as plt


def sigmoid(x):
    """シグモイド関数"""
    return 1 / (1 + np.exp(-x))


# x軸の範囲を設定
x = np.linspace(-10, 10, 100)

# シグモイド関数を計算
y = sigmoid(x)

# グラフを描画
plt.figure(figsize=(8, 6))
plt.rcParams.update({'font.size': 16})  # 全体の文字サイズを大きく
plt.plot(x, y, 'b-', linewidth=2, label='sigmoid(x) = 1 / (1 + e^(-x))')
plt.axhline(y=0.5, color='gray', linestyle='--', alpha=0.5)
plt.axvline(x=0, color='gray', linestyle='--', alpha=0.5)
plt.xlabel('x', fontsize=22)
plt.ylabel('sigmoid(x)', fontsize=22)
plt.tick_params(labelsize=16)  # 軸目盛りの文字サイズ
#plt.title('シグモイド関数')
#plt.legend()
plt.grid(True, alpha=0.3)
plt.ylim(-0.1, 1.1)
plt.show()
