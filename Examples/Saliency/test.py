import numpy as np
import matplotlib.pyplot as plt

# 1. 读取数据
filename = '/home/nuc/orb_slam3/ORBSLAM/ORBSLAM/Examples/Saliency/kitti_saliency.txt'
try:
    data = np.loadtxt(filename)
except:
    print("找不到文件")
    exit()

x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

# 2. 寻找最大值点（用于验证）
max_idx = np.argmax(z)
max_x = x[max_idx]
max_y = y[max_idx]
max_z = z[max_idx]
print(f"检测到最高点: Z={max_z:.2f} 位于 X={max_x:.2f}, Y={max_y:.2f}")

# 3. 绘制 2D 平面图 (X vs Y, Color=Z)
plt.figure(figsize=(12, 10))

# s=15 是点的大小，越小分辨率越高
# alpha=0.8 是透明度，防止点太密集完全盖住
# cmap='jet' 或者 'viridis' 是色卡，jet 对高低值的红蓝对比更强烈
sc = plt.scatter(x, y, c=z, cmap='jet', s=15, alpha=0.9, edgecolors='none')

# 添加颜色条
cbar = plt.colorbar(sc)
cbar.set_label('Z Value (Height)')

# 4. 特别标注出最高点（画个红圈）
plt.scatter(max_x, max_y, s=300, facecolors='none', edgecolors='black', linewidth=2, label='Peak Location')
plt.text(max_x, max_y, f'  Max: {max_z:.1f}', color='black', fontsize=12, fontweight='bold')

# 5. 设置显示范围和标签
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title(f'2D Heatmap (Top View)\nColor represents Z value')
plt.axis('equal') # 保证 X 和 Y 轴比例一致，图形不变形
plt.legend()
plt.grid(True, linestyle='--', alpha=0.5)

plt.show()