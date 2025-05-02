import csv
from collections import Counter
from itertools import combinations

import cv2
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import yaml
from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon
from scipy.spatial import Delaunay


def crosses_obstacle(p1, p2, binary_map, resolution, origin):
    # Convert world → pixel coordinates
    def world_to_px(pt):
        x_pix = int((pt[0] - origin[0]) / resolution)
        y_pix = int((origin[1] + binary_map.shape[0]*resolution - pt[1]) / resolution)
        return x_pix, y_pix

    x0, y0 = world_to_px(p1)
    x1, y1 = world_to_px(p2)

    # Bresenham-like sampling
    num = int(np.hypot(x1 - x0, y1 - y0))
    xs = np.linspace(x0, x1, num=num)
    ys = np.linspace(y0, y1, num=num)

    for x, y in zip(xs, ys):
        xi, yi = int(round(x)), int(round(y))
        if 0 <= xi < binary_map.shape[1] and 0 <= yi < binary_map.shape[0]:
            if binary_map[yi, xi] == 1:
                return True  # intersects obstacle
    return False

### Load Map
map_dir = "../resource/"
map_name = "map_house"

map_img = cv2.imread(map_dir + map_name + '.pgm', cv2.IMREAD_GRAYSCALE)
binary_map = (map_img < 250).astype(np.uint8)  # 1 = occupied

# Image to world coords
with open(map_dir + map_name + '.yaml', 'r') as f:
    map_info = yaml.safe_load(f)

resolution = map_info['resolution']
origin = map_info['origin']  # [x, y, yaw]

### Defining room points

# Room A
room_A = np.array([
    [-7.29, -3.77], [-5.35, -3.80], [-5.27, -2.56], [-5.77, -2.52],
    [-5.80, -1.22], [-5.29, -1.20], [-7.27, 0.65], [-6.54, 0.70],
    [-5.96, 0.70],  [-5.35, 0.69], [-5.27, -0.48], [-7.05, -1.48],
    [-5.77, -3.62]
], dtype=np.float32)

# Room B
room_B = np.array([
    [-7.29, 1.69], [-7.28, 4.76], [-5.91, 4.79], [-5.91, 5.11], [-5.28, 5.14],
    [-5.28, 4.53], [-5.29, 2.22], [-5.27, 1.24], [-5.88, 1.29], [-6.70, 1.32],
    [-6.78, 1.67], [-5.14, 2.43]
], dtype=np.float32)

# Room C
room_C = np.array([
    [-4.88, 4.52], [-4.91, 2.18], [-4.89, 0.11], [-4.30, 4.61], [-4.27, 5.11],
    [-3.62, 0.12], [-3.45, 1.90], [-3.50, 3.11], [-1.81, 3.18], [-1.71, 2.01],
    [-0.18, 5.09], [-0.17, 0.04], [-0.34, 3.34], [-0.35, 1.13], [-4.80, 1.02],
    [-2.58, 1.88], [-0.39, 2.43]
], dtype=np.float32)

# Room D
room_D = np.array([
    [2.16, 4.92], [0.14, 5.01], [0.13, 1.17], [1.55, 1.11],
    [1.53, 2.26], [2.04, 2.32], [2.14, 4.19], [0.19, 3.02], [0.19, 1.85]
], dtype=np.float32)

# Room E
room_E = np.array([
    [-0.30, 0.67], [-0.17, 0.04], [2.31, 0.67], [2.33, 0.00], [0.94, 0.64],
    [0.72, 0.27], [1.84, 0.47], [0.39, 0.59], [1.04, -0.02], [1.60, 0.63]
], dtype=np.float32)

# Room F
room_F = np.array([
    [2.31, 0.67], [2.33, 0.00], [2.31, 0.67], [2.33, 0.00],
    [2.48, 4.12], [2.48, 5.01], [4.18, 4.95], [4.20, 4.53],
    [6.18, 4.45], [6.24, 4.84], [7.31, 4.80], [3.84, 3.45],
    [5.92, 3.45], [3.76, 1.85], [5.93, 1.85], [5.66, -0.05],
    [6.5, 0.00], [7.21, -0.01], [2.59, 2.69], [7.12, 2.61],
    [3.58, 1.11], [3.16, 0.27]
], dtype=np.float32)

# Room G
room_G = np.array([
    [5.79, -0.33], [6.50, -0.31], [4.98, -0.42], [7.22, -0.46],
    [4.94, -4.91], [7.20, -4.91], [5.62, -1.80], [6.49, -1.78],
    [5.61, -3.49], [6.59, -3.52], [5.08, -2.87], [7.07, -2.62],
    [2.63, 0.09],[2.64, 0.86],[4.33, 0.09]
], dtype=np.float32)

labels = np.array(
    ['A'] * len(room_A) +
    ['B'] * len(room_B) +
    ['C'] * len(room_C) +
    ['D'] * len(room_D) +
    ['E'] * len(room_E) +
    ['F'] * len(room_F) +
    ['G'] * len(room_G)
)

points_world = np.vstack([
    room_A,
    room_B,
    room_C,
    room_D,
    room_E,
    room_F,
    room_G
])

### Triangulation

tri = Delaunay(points_world)

valid_triangles = []

for tri_indices in tri.simplices:
    p1, p2, p3 = points_world[tri_indices]

    if any([
        crosses_obstacle(p1, p2, binary_map, resolution, origin),
        crosses_obstacle(p2, p3, binary_map, resolution, origin),
        crosses_obstacle(p3, p1, binary_map, resolution, origin)
    ]):
        continue  # reject triangle
    valid_triangles.append(tri_indices)

valid_triangles = np.array(valid_triangles)

### Assign rooms to triangles

# Map room to color
room_to_color = {
    'A': 'red', 'B': 'blue', 'C': 'green',
    'D': 'orange', 'E': 'purple', 'F': 'brown', 'G': 'cyan', 'obs': 'black'
}

# Determine triangle labels and colors
valid_tri_labels = []
triangle_colors = []
 
for simplex in valid_triangles:
    tri_labels = labels[simplex]
    dominant = Counter(tri_labels).most_common(1)[0][0]
    valid_tri_labels.append(dominant)
    triangle_colors.append(room_to_color[dominant])

valid_tri_labels[19] = 'F'
triangle_colors[19] = 'brown'
valid_tri_labels[15] = 'F'
triangle_colors[15] = 'brown'
valid_tri_labels[17] = 'F'
triangle_colors[17] = 'brown'

valid_tri_labels[55] = "obs"
triangle_colors[55] = "black"
valid_tri_labels[47] = "obs"
triangle_colors[47] = "black"

### Plotting
# Prepare image as background
extent = [
    origin[0],
    origin[0] + map_img.shape[1] * resolution,
    origin[1],
    origin[1] + map_img.shape[0] * resolution
]

# plt.imshow(
#     np.flipud(map_img),  # flip vertically to match coordinate orientation
#     cmap='gray',
#     extent=extent,
#     origin='lower'
# )

# # Plot triangulation
# plt.triplot(points_world[:, 0], points_world[:, 1], valid_triangles, color='red', linewidth=0.3)
# plt.plot(points_world[:, 0], points_world[:, 1], '.', markersize=0.5, color='blue')

# plt.gca().set_aspect('equal')
# plt.title("Delaunay Triangulation Overlaid on Map")

# plt.show()


## Segmentation Plot
fig, ax = plt.subplots()

# Show the map image as background
ax.imshow(
    np.flipud(map_img),
    cmap='gray',
    extent=extent,
    origin='lower'
)

# Add colored triangle patches
patches = []
for simplex in valid_triangles:
    triangle = Polygon(points_world[simplex], True)
    patches.append(triangle)

p = PatchCollection(patches, facecolor=triangle_colors, edgecolor='k', alpha=0.5)
ax.add_collection(p)

# Add triangle numbers
for i, simplex in enumerate(valid_triangles):
    centroid = np.mean(points_world[simplex], axis=0)
    ax.text(centroid[0], centroid[1], str(i), fontsize=6, ha='center', va='center')

# Add point markers
ax.plot(points_world[:, 0], points_world[:, 1], 'ko', markersize=2)

ax.set_aspect('equal')
# plt.title("Filtered Triangles Over Map (Colored by Room)")
# plt.show()

# ### Save triangle labels
# with open("tri_labels.csv", "w", newline='') as f:
#     writer = csv.writer(f)
#     writer.writerow(["triangle_index", "room_label"])  # header
#     for i, label in enumerate(valid_tri_labels):
#         writer.writerow([i, label])

### Build triangle graph

# Build triangle adjacency graph
G_tri = nx.Graph()

# Step 1: Add each triangle as a node (indexed by triangle ID)
for i in range(len(valid_triangles)):
    G_tri.add_node(i)

# Step 2: Compare triangle pairs for shared edges
for i, tri_i in enumerate(valid_triangles):
    edges_i = {frozenset(e) for e in combinations(tri_i, 2)}
    for j in range(i + 1, len(valid_triangles)):
        tri_j = valid_triangles[j]
        edges_j = {frozenset(e) for e in combinations(tri_j, 2)}
        if edges_i & edges_j:  # shared edge
            G_tri.add_edge(i, j)

centroids = np.array([points_world[tri].mean(axis=0) for tri in valid_triangles])
pos = {i: tuple(c) for i, c in enumerate(centroids)}

# fig, ax = plt.subplots()

# # Show the background map image
# ax.imshow(
#     np.flipud(map_img),
#     cmap='gray',
#     extent=extent,
#     origin='lower'
# )

# Draw the triangle graph using centroids
nx.draw(
    G_tri,
    pos=pos,
    ax=ax,
    node_size=20,
    node_color='red',
    edge_color='cyan',
    with_labels=True,
    font_size=6
)

ax.set_aspect('equal')
plt.title("Triangle Graph Overlaid on Map")
plt.show()
### Save triangle graph

# # Set the initial triangle index (e.g., arbitrarily to 41)
# initial_node = 41

# # Compute triangle centroids
# centroids = np.array([points_world[tri].mean(axis=0) for tri in valid_triangles])
# centroid_dict = {f"q{i}": tuple(np.round(pt, 3)) for i, pt in enumerate(centroids)}

# # Build nodes section
# nodes = {}
# for qname, loc in centroid_dict.items():
#     nodes[qname] = {
#         "prop": {},  # empty label set
#         "location": loc
#     }

# # Build edges section
# edges = []
# for u, v in G_tri.edges():
#     edges.append([f"q{u}", f"q{v}", {"weight": 1}])

# # Assemble full structure
# graph_data = {
#     "!Ts": None,
#     "name": "Triangle Adjacency Graph",
#     "directed": False,
#     "multi": False,
#     "init": [f"q{initial_node}"],
#     "final": [],
#     "graph": {
#         "nodes": nodes,
#         "edges": edges
#     }
# }

# # Save to YAML
# with open("triangle_graph.yaml", "w") as f:
#     yaml.dump(graph_data, f, sort_keys=False, default_flow_style=False)

# Build node label map from valid_tri_labels
# Start building the YAML string manually
# Compute centroids of triangles
centroids = [points_world[tri].mean(axis=0) for tri in valid_triangles]

# Start building the YAML text
lines = []
lines.append("!Ts")
lines.append("name: Triangle Adjacency Graph")
lines.append("directed: false")
lines.append("multi: false")
lines.append("init: ['q41']")
lines.append("final: []")
lines.append("graph:")
lines.append("  nodes:")

# Add node definitions with centroids as location
for i, (room, centroid) in enumerate(zip(valid_tri_labels, centroids)):
    x, y = round(float(centroid[0]), 3), round(float(centroid[1]), 3)
    lines.append(f"    q{i}:")
    lines.append(f"      prop: !!set {{'{room}': null}}")
    lines.append(f"      location: !!python/tuple [{x}, {y}]")

# Add edge definitions
lines.append("  edges:")
for u, v in G_tri.edges():
    lines.append(f"  - [q{u}, q{v}, {{'weight': 1}}]")

# Join and save
yaml_text = '\n'.join(lines)

with open("triangle_graph.yaml", "w") as f:
    f.write(yaml_text)

# Optionally print
print(yaml_text)

