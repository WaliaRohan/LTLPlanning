# Write code to perform delauny traingulation on environment

# TODO: How do we provide the environment?

from scipy.spatial import Delaunay
import numpy as np
import matplotlib.pyplot as plt
import cv2
import yaml

# Load Map
map_img = cv2.imread('../resource/turtlebot3_world.pgm', cv2.IMREAD_GRAYSCALE)
binary_map = (map_img < 250).astype(np.uint8)  # 1 = occupied

# Extract free space points
ys, xs = np.where(binary_map == 0)  # free pixels
points_px = np.stack((xs, ys), axis=-1)  # (N, 2)

# Image to world coords
with open('../resource/turtlebot3_world.yaml', 'r') as f:
    map_info = yaml.safe_load(f)

resolution = map_info['resolution']
origin = map_info['origin']  # [x, y, yaw]

points_world = np.zeros_like(points_px, dtype=np.float32)
points_world[:, 0] = points_px[:, 0] * resolution + origin[0]
points_world[:, 1] = (binary_map.shape[0] - points_px[:, 1]) * resolution + origin[1]

# Subsample points
num_samples = int(0.5 * len(points_world))
indices = np.random.choice(len(points_world), num_samples, replace=False)
points_world = points_world[indices]

tri = Delaunay(points_world)
# --- Plotting ---
# Prepare image as background
extent = [
    origin[0],
    origin[0] + map_img.shape[1] * resolution,
    origin[1],
    origin[1] + map_img.shape[0] * resolution
]

plt.imshow(
    np.flipud(map_img),  # flip vertically to match coordinate orientation
    cmap='gray',
    extent=extent,
    origin='lower'
)

# Plot triangulation
plt.triplot(points_world[:, 0], points_world[:, 1], tri.simplices, color='red', linewidth=0.3)
plt.plot(points_world[:, 0], points_world[:, 1], '.', markersize=0.5, color='blue')

plt.gca().set_aspect('equal')
plt.title("Delaunay Triangulation Overlaid on Map")
plt.show()

# Get Create vector fields for starting and ending point
