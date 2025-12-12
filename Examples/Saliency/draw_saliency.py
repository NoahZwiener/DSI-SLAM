import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.axes_grid1 import make_axes_locatable  # 关键库：用于对齐Colorbar
import cv2
from scipy.ndimage import gaussian_filter
from matplotlib.ticker import MaxNLocator

def generate_saliency_visualization(image_path, data_path):
    # 1. 读取图像
    # ---------------------------------------------------------
    # 使用 IMREAD_ANYCOLOR 确保兼容灰度和彩色
    img = cv2.imread(image_path, cv2.IMREAD_ANYCOLOR)
    
    if img is None:
        print(f"Error: 无法读取图片: {image_path}")
        return

    # 获取图像尺寸
    if len(img.shape) == 2:
        h, w = img.shape
        # 如果是单通道灰度图，转为RGB方便后续画图（叠加颜色）
        img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    else:
        h, w, c = img.shape
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    print(f"Image Size: Width={w}, Height={h}")

    # 2. 读取数据并清洗
    # ---------------------------------------------------------
    try:
        data = np.loadtxt(data_path)
    except Exception as e:
        print(f"Error: 无法读取数据文件 {data_path}: {e}")
        return

    if data.ndim == 1: data = data.reshape(1, -1)
    if data.shape[0] == 0:
        print("Error: 数据文件为空")
        return

    # 原始数据
    raw_x = data[:, 0]
    raw_y = data[:, 1]
    raw_s = data[:, 2]

    print(f"Total Points Loaded: {len(raw_x)}")
    print(f"Max X in data: {raw_x.max():.1f}, Max Y in data: {raw_y.max():.1f}")

    # --- 关键修复：过滤越界点 ---
    # 有时候边缘的特征点坐标可能会略微超出 w-1 或 h-1
    valid_mask = (raw_x >= 0) & (raw_x < w) & (raw_y >= 0) & (raw_y < h)
    
    points_x = raw_x[valid_mask].astype(int)
    points_y = raw_y[valid_mask].astype(int)
    saliency_values = raw_s[valid_mask]

    removed_count = len(raw_x) - len(points_x)
    if removed_count > 0:
        print(f"Warning: 自动移除了 {removed_count} 个超出图片范围的异常点")
    # points_x = raw_x
    # points_y = raw_y
    # saliency_values = raw_s

    sort_idx = np.argsort(saliency_values)
    
    points_x = points_x[sort_idx]
    points_y = points_y[sort_idx]
    saliency_values = saliency_values[sort_idx]
    # 3. 生成热力图
    # ---------------------------------------------------------
    saliency_map = np.zeros((h, w), dtype=np.float32)
    # 填充值
    for x, y, s in zip(points_x, points_y, saliency_values):
        saliency_map[y, x] = max(saliency_map[y, x], s)

    # 高斯模糊
    smooth_saliency = gaussian_filter(saliency_map, sigma=25)
    # 归一化
    if smooth_saliency.max() > 0:
        smooth_saliency /= smooth_saliency.max()

    # 4. 绘图
    # ---------------------------------------------------------
    #fig = plt.figure(figsize=(16, 5))   #Euroc
    fig = plt.figure(figsize=(20, 4.5))  #KITTI
    # --- 图1: 原始图片 + 特征点 ---
    ax1 = fig.add_subplot(131)
    # 明确显示灰度底图
    ax1.imshow(img_rgb) 
    # 绘制点，s=5是点的大小，linewidths=0去掉点的边框使显示更清晰
    sc = ax1.scatter(points_x, points_y, c=saliency_values, cmap='jet', s=10, alpha=0.8, linewidths=0)
    ax1.set_title("Features Map", y=1.0)
    # 强制限制坐标轴范围，防止点“飞”出去
    ax1.set_xlim(0, w)
    ax1.set_ylim(h, 0) # 注意：图像坐标系Y轴向下，这里设定h在下，0在上
    ax1.axis('off')

    # --- 图2: 热力图叠加 ---
    ax2 = fig.add_subplot(132)
    ax2.imshow(img_rgb)
    heatmap = ax2.imshow(smooth_saliency, cmap='jet', alpha=0.5)
    ax2.set_title("Saliency Heatmap",y=1.0)
    ax2.axis('off')


    divider = make_axes_locatable(ax2)
    # append_axes("right", size="5%", pad=0.05) 会在右侧“切”出一块区域放colorbar
    # 这样colorbar的高度就和ax2的主图完全一致了
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = plt.colorbar(heatmap, cax=cax)
    cbar.set_label('Saliency Norm')

    #--- 图3: 3D 曲面 ---
    ax3 = fig.add_subplot(133, projection='3d')
    stride = 8 # 降采样，值越大网格越稀疏，画图越快
    X = np.arange(0, w, stride)
    Y = np.arange(0, h, stride)
    X, Y = np.meshgrid(X, Y)
    Z = smooth_saliency[::stride, ::stride]
    
    # 确保Z的尺寸和X,Y匹配 (处理除法取整导致的边缘问题)
    # 通常 meshgrid 后 X, Y 形状由 ranges 决定
    # 我们切片 smooth_saliency 时要小心
    real_h, real_w = Z.shape
    X = X[:real_h, :real_w]
    Y = Y[:real_h, :real_w]

    surf = ax3.plot_surface(X, Y, Z, cmap='jet', linewidth=0, antialiased=False)

    # nbins=4 表示最多允许 4 个刻度 (例如 0, 100, 200, 300)
    ax3.yaxis.set_major_locator(MaxNLocator(nbins=4))
    
    # 2. 同理，X 轴虽然长，但刻度太密也不好看，也可以限制一下
    ax3.xaxis.set_major_locator(MaxNLocator(nbins=6))

    # 3. 调整刻度字体大小和旋转角度 (防止和轴线打架)
    ax3.tick_params(axis='y', labelsize=8, rotation=-15, pad=0)
    ax3.tick_params(axis='x', labelsize=8, pad=0)
    ax3.tick_params(axis='z', labelsize=8)

    z_visual_height = h * 1.5 # 调节这个系数来控制“山峰”显示的相对高度
    ax3.set_box_aspect((w, h, z_visual_height))

    #ax3.view_init(elev=50, azim=-60)
    ax3.view_init(elev=40, azim=-70)
    ax3.set_title("3D Surface")
    ax3.set_title("3D Surface", y=1.05)
    ax3.set_zlim(0, 1.0)
    ax3.invert_yaxis() # 对齐图像坐标系

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python view_saliency.py <image_path> <txt_path>")
        sys.exit(1)

    img_file = sys.argv[1]
    txt_file = sys.argv[2]
    
    generate_saliency_visualization(img_file, txt_file)