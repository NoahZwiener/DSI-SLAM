import sys
import os
import subprocess
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.axes_grid1 import make_axes_locatable  
import cv2
from scipy.ndimage import gaussian_filter
from matplotlib.ticker import MaxNLocator
plt.rcParams["axes.titlesize"] = 16
plt.rcParams["axes.labelsize"] = 13
plt.rcParams["xtick.labelsize"] = 11
plt.rcParams["ytick.labelsize"] = 11
plt.rcParams["legend.fontsize"] = 11

def show_features_map(img_rgb, points_x, points_y, saliency_values, w, h):
    fig, ax = plt.subplots(figsize=(8, 6), num="Features Map")
    ax.imshow(img_rgb)
    ax.scatter(points_x, points_y, c=saliency_values, cmap='jet', s=10, alpha=0.8, linewidths=0)
    ax.set_title("Features Map")
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.axis('off')
    fig.tight_layout()
    plt.show()


def show_heatmap(img_rgb, smooth_saliency):
    fig, ax = plt.subplots(figsize=(8, 6), num="Saliency Heatmap")
    ax.imshow(img_rgb)
    heatmap = ax.imshow(smooth_saliency, cmap='jet', alpha=0.5)
    ax.set_title("Saliency Heatmap")
    ax.axis('off')

    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = plt.colorbar(heatmap, cax=cax)
    cbar.set_label('Saliency Norm')

    fig.tight_layout()
    plt.show()


def show_surface3d(smooth_saliency, w, h):
    fig = plt.figure(figsize=(9, 6), num="3D Surface")
    ax = fig.add_subplot(111, projection='3d')

    stride = 8
    X = np.arange(0, w, stride)
    Y = np.arange(0, h, stride)
    X, Y = np.meshgrid(X, Y)
    Z = smooth_saliency[::stride, ::stride]

    real_h, real_w = Z.shape
    X = X[:real_h, :real_w]
    Y = Y[:real_h, :real_w]

    ax.plot_surface(X, Y, Z, cmap='jet', linewidth=0, antialiased=False)
    ax.yaxis.set_major_locator(MaxNLocator(nbins=4))
    ax.xaxis.set_major_locator(MaxNLocator(nbins=6))
    ax.tick_params(axis='y', labelsize=8, rotation=-15, pad=0)
    ax.tick_params(axis='x', labelsize=8, pad=0)
    ax.tick_params(axis='z', labelsize=8)

    z_visual_height = h * 1.5
    ax.set_box_aspect((w, h, z_visual_height))
    ax.view_init(elev=40, azim=-70)
    ax.set_title("3D Surface", y=1.05)
    ax.set_zlim(0, 1.0)
    ax.invert_yaxis()

    fig.tight_layout()
    plt.show()


def load_and_prepare(image_path, data_path):
    img = cv2.imread(image_path, cv2.IMREAD_ANYCOLOR)

    if img is None:
        print(f"Error: 无法读取图片: {image_path}")
        return None

    if len(img.shape) == 2:
        h, w = img.shape
        img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    else:
        h, w, _ = img.shape
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    print(f"Image Size: Width={w}, Height={h}")

    try:
        data = np.loadtxt(data_path)
    except Exception as e:
        print(f"Error: 无法读取数据文件 {data_path}: {e}")
        return None

    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[0] == 0:
        print("Error: 数据文件为空")
        return None

    raw_x = data[:, 0]
    raw_y = data[:, 1]
    raw_s = data[:, 2]

    print(f"Total Points Loaded: {len(raw_x)}")
    print(f"Max X in data: {raw_x.max():.1f}, Max Y in data: {raw_y.max():.1f}")

    valid_mask = (raw_x >= 0) & (raw_x < w) & (raw_y >= 0) & (raw_y < h)

    points_x = raw_x[valid_mask].astype(int)
    points_y = raw_y[valid_mask].astype(int)
    saliency_values = raw_s[valid_mask]

    removed_count = len(raw_x) - len(points_x)
    if removed_count > 0:
        print(f"Warning: 自动移除了 {removed_count} 个超出图片范围的异常点")

    sort_idx = np.argsort(saliency_values)
    points_x = points_x[sort_idx]
    points_y = points_y[sort_idx]
    saliency_values = saliency_values[sort_idx]

    saliency_map = np.zeros((h, w), dtype=np.float32)
    for x, y, s in zip(points_x, points_y, saliency_values):
        saliency_map[y, x] = max(saliency_map[y, x], s)

    smooth_saliency = gaussian_filter(saliency_map, sigma=25)
    if smooth_saliency.max() > 0:
        smooth_saliency /= smooth_saliency.max()

    return img_rgb, points_x, points_y, saliency_values, smooth_saliency, w, h


def run_single_view(image_path, data_path, view):
    prepared = load_and_prepare(image_path, data_path)
    if prepared is None:
        return 1

    img_rgb, points_x, points_y, saliency_values, smooth_saliency, w, h = prepared

    if view == "features":
        show_features_map(img_rgb, points_x, points_y, saliency_values, w, h)
    elif view == "heatmap":
        show_heatmap(img_rgb, smooth_saliency)
    elif view == "surface3d":
        show_surface3d(smooth_saliency, w, h)
    else:
        print(f"Error: 不支持的 view 类型: {view}")
        return 1

    return 0

def generate_saliency_visualization(image_path, data_path):
    # Matplotlib GUI 不能安全地在子线程中初始化，这里改为子进程并行显示。
    script_path = os.path.abspath(__file__)
    views = ["features", "heatmap", "surface3d"]
    procs = []

    for view in views:
        cmd = [sys.executable, script_path, image_path, data_path, "--view", view]
        procs.append(subprocess.Popen(cmd))

    final_code = 0
    for p in procs:
        code = p.wait()
        if code != 0:
            final_code = code

    return final_code


def build_parser():
    parser = argparse.ArgumentParser(description="Saliency 可视化")
    parser.add_argument("image_path", help="输入图像路径")
    parser.add_argument("txt_path", help="saliency 文本数据路径")
    parser.add_argument(
        "--view",
        choices=["all", "features", "heatmap", "surface3d"],
        default="all",
        help="选择绘制视图，默认 all",
    )
    return parser

if __name__ == "__main__":
    args = build_parser().parse_args()
    if args.view == "all":
        raise SystemExit(generate_saliency_visualization(args.image_path, args.txt_path))
    raise SystemExit(run_single_view(args.image_path, args.txt_path, args.view))
