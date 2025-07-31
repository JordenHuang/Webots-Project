import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

VIEW = False

point_cloud = np.load('3dPointCloud.npy')

if VIEW == True:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(point_cloud[:, :, 0], -point_cloud[:, :, 2], point_cloud[:, :, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Z')
    ax.set_zlabel('Y')
    plt.title('3D Point Cloud Visualization with Matplotlib')
    plt.show()

print(point_cloud.shape)

# 過濾不合法的點（NaN、inf、負Z）
mask = np.isfinite(point_cloud[:, :, 2]) & (point_cloud[:, :, 2] > 0)
points_valid = point_cloud[mask]  # shape: (N, 3)

# 取得 X-Z (忽略高度)
points_2d = points_valid[:, [0, 2]]  # shape: (N, 2)

cell_size = 0.1  # 每格 10cm
x_min, z_min = points_2d.min(axis=0)
x_max, z_max = points_2d.max(axis=0)

width_cells  = int(np.ceil((x_max - x_min) / cell_size))
height_cells = int(np.ceil((z_max - z_min) / cell_size))

# 建立空白地圖，初始為 -1（未知）
grid_map = np.full((height_cells, width_cells), -1, dtype=np.int8)

# 將點轉為 cell 座標
grid_x = ((points_2d[:, 0] - x_min) / cell_size).astype(int)
grid_y = ((points_2d[:, 1] - z_min) / cell_size).astype(int)
print(f"len: {len(grid_x)}, {len(grid_y)}")

# 標記 cell 為可達（或障礙物）
for x, y in zip(grid_x, grid_y):
    if 0 <= y < height_cells and 0 <= x < width_cells:
        grid_map[y, x] = 0  # 你可以先標為可達區域

for x, y, point in zip(grid_x, grid_y, points_valid):
    if 0 <= y < height_cells and 0 <= x < width_cells:
        if point[1] > 0.1:  # 高度大於10cm
            grid_map[y, x] = 1  # 有障礙物
        else:
            grid_map[y, x] = 0  # 可走

plt.imshow(grid_map, cmap='gray', origin='lower')
plt.title("2D Occupancy Grid Map")
plt.xlabel("X (grid)")
plt.ylabel("Z (grid)")
plt.show()