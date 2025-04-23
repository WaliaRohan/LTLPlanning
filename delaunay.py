# Write code to perform delauny traingulation on environment

# TODO: How do we provide the environment?

from scipy.spatial import Delaunay
import numpy as np
import matplotlib.pyplot as plt
import cv2
import yaml

# Load Map
map_img = cv2.imread('turtlebot3_world.pgm', cv2.IMREAD_GRAYSCALE)
binary_map = (map_img < 250).astype(np.uint8)  # 1 = occupied

# Extract free space points
ys, xs = np.where(binary_map == 0)  # free pixels
points_px = np.stack((xs, ys), axis=-1)  # (N, 2)

# Image to world coords
with open('turtlebot3_world.yaml', 'r') as f:
    map_info = yaml.safe_load(f)

resolution = map_info['resolution']
origin = map_info['origin']  # [x, y, yaw]
points_world = np.zeros_like(points_px, dtype=np.float32)
points_world[:, 0] = points_px[:, 0] * resolution + origin[0]
points_world[:, 1] = (binary_map.shape[0] - points_px[:, 1]) * resolution + origin[1]

tri = Delaunay(points_world)
plt.triplot(points_world[:, 0], points_world[:, 1], tri.simplices)
plt.plot(points_world[:, 0], points_world[:, 1], '.', markersize=0.5)
plt.gca().set_aspect('equal')
plt.show()


# points = np.array([[0, 0], [0, 1.1], [1, 0], [1, 1]])
# tri = Delaunay(points)

# plt.triplot(points[:,0], points[:,1], tri.simplices)
# plt.plot(points[:,0], points[:,1], 'o')
# plt.show()

# TODO How do we create vector fields from the output of delaunay?


