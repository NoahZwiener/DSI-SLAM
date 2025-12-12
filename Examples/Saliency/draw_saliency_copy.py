import sys
import numpy as np
import matplotlib.pyplot as plt
import cv2
# 使用方法：
# python3 draw_saliency_copy.py ../../dataset/kitti/odometry/data_odometry_gray/dataset/sequences/05/image_0/000147.png  output.txt
def generate_layered_visualization(image_path, data_path):
    # 1. 读取图像
    # ---------------------------------------------------------
    img = cv2.imread(image_path, cv2.IMREAD_ANYCOLOR)
    
    if img is None:
        print(f"Error: 无法读取图片: {image_path}")
        return

    # 颜色转换：BGR -> RGB
    if len(img.shape) == 2:
        h, w = img.shape
        img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    else:
        h, w, c = img.shape
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    print(f"Image Size: Width={w}, Height={h}")

    # 2. 读取数据 (兼容空格或逗号分隔)
    # ---------------------------------------------------------
    try:
        # 尝试默认读取 (空格分隔)
        data = np.loadtxt(data_path)
    except ValueError:
        try:
            # 尝试逗号分隔
            data = np.loadtxt(data_path, delimiter=',')
        except Exception as e:
            print(f"Error: 无法解析数据文件: {e}")
            return
    except Exception as e:
        print(f"Error: 无法读取数据文件: {e}")
        return

    # 处理只有一行数据的情况
    if data.ndim == 1: 
        data = data.reshape(1, -1)
    
    if data.shape[0] == 0:
        print("Error: 数据文件为空")
        return

    # 3. 解析四列数据
    # 格式: x, y, saliency, level
    # ---------------------------------------------------------
    raw_x = data[:, 0]
    raw_y = data[:, 1]
    raw_s = data[:, 2] # 显著性
    raw_l = data[:, 3] # 层级 (Level)

    print(f"Total Points: {len(raw_x)}")
    
    # 确定层级范围 (通常是 0-7)
    # 我们强制设定为 0-7 以确保格式统一，或者根据数据动态获取
    # target_levels = range(8) # 0, 1, ..., 7
    target_levels = [7]
    # 获取全局显著性最大最小值，用于统一颜色标准
    # 这样 Level 0 的红色和 Level 7 的红色代表的意义是一样的
    global_vmin = raw_s.min()
    global_vmax = raw_s.max()
    print(f"Saliency Range: {global_vmin} ~ {global_vmax}")

    # 4. 绘图：垂直堆叠 (Vertical Stack)
    # ---------------------------------------------------------
    num_levels = len(target_levels)
    
    # 动态计算画布高度：
    # KITTI图片很宽(20)，高度假设每个子图给 3.5 inch，总高度就是 num_levels * 3.5
    fig_height = 3.5 * num_levels 
    fig, axes = plt.subplots(nrows=num_levels, ncols=1, figsize=(20, fig_height))
    
    # 如果只有一层，axes不是列表，转为列表方便循环
    if num_levels == 1:
        axes = [axes]

    print("开始绘制分层图...")

    for i, level in enumerate(target_levels):
        ax = axes[i]
        
        # --- (A) 筛选当前层级的数据 ---
        mask = (raw_l == level)
        points_x = raw_x[mask]
        points_y = raw_y[mask]
        saliency_val = raw_s[mask]
        
        # --- (B) 绘制底图 ---
        ax.imshow(img_rgb)
        
        # --- (C) 绘制特征点 ---
        if len(points_x) > 0:
            # [重要] 排序：显著性低的先画，显著性高(红)的后画，防止遮挡
            sort_idx = np.argsort(saliency_val)
            points_x = points_x[sort_idx]
            points_y = points_y[sort_idx]
            saliency_val = saliency_val[sort_idx]

            # 绘制散点
            # vmin/vmax 使用全局变量，保证颜色统一
            sc = ax.scatter(points_x, points_y, 
                            c=saliency_val, 
                            cmap='jet', 
                            vmin=global_vmin, vmax=global_vmax,
                            s=15, alpha=0.9, linewidths=0)
        
        # --- (D) 美化设置 ---
        ax.set_title(f"Level {level} (Points: {len(points_x)})", fontsize=14, y=1.02)
        ax.set_xlim(0, w)
        ax.set_ylim(h, 0) # 图像坐标系
        ax.axis('off')

    # 调整布局
    plt.tight_layout()
    
    # 如果想保存图片，取消下面这行的注释
    # plt.savefig("layered_features.png", dpi=100)
    
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python view_saliency_layered.py <image_path> <txt_path>")
        sys.exit(1)

    img_file = sys.argv[1]
    txt_file = sys.argv[2]
    
    generate_layered_visualization(img_file, txt_file)