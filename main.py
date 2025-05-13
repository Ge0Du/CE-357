import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import seaborn as sns
import pandas as pd
import random
from collections import defaultdict

# === Step 1: Define More Macros and Clusters ===

# Expand sample macros and clusters
macros = [f'M{i+1}' for i in range(12)]
clusters = [f'C{i+1}' for i in range(30)]

# Randomly generate connectivity weights
np.random.seed(42)
connectivity = {cluster: {macro: np.random.randint(0, 100) for macro in macros} for cluster in clusters}

# === Step 2: Assign each cluster to the macro with highest connectivity ===

assigned_macro = {}
macro_to_clusters = defaultdict(list)

for cluster in clusters:
    best_macro = max(connectivity[cluster], key=connectivity[cluster].get)
    assigned_macro[cluster] = best_macro
    macro_to_clusters[best_macro].append(cluster)

# === Step 3: Generate Random Positions and Areas for Clusters ===

cluster_positions = {}
cluster_areas = {}
for cluster in clusters:
    cluster_positions[cluster] = np.random.rand(2) * 100
    cluster_areas[cluster] = np.random.randint(100, 500)

# === Step 4: Compute Macro Centroids and Bounding Boxes ===

macro_positions = {}
cluster_bounding_boxes = {}

for macro in macros:
    assigned_clusters = macro_to_clusters[macro]
    if not assigned_clusters:
        macro_positions[macro] = np.random.rand(2) * 100
        continue

    total_weight = sum(connectivity[c][macro] for c in assigned_clusters)
    weighted_x = sum(cluster_positions[c][0] * connectivity[c][macro] for c in assigned_clusters)
    weighted_y = sum(cluster_positions[c][1] * connectivity[c][macro] for c in assigned_clusters)
    centroid = np.array([weighted_x / total_weight, weighted_y / total_weight])
    macro_positions[macro] = centroid

    total_area = sum(cluster_areas[c] for c in assigned_clusters)
    target_util = 0.7
    box_area = total_area / target_util
    box_width = box_height = np.sqrt(box_area)
    box_corner = centroid - box_width / 2
    cluster_bounding_boxes[macro] = (box_corner, box_width)

# === Step 5: Assign Unique Colors to Macros ===

macro_list = list(macro_to_clusters.keys())
num_macros = len(macro_list)
colormap = cm.get_cmap('tab20', num_macros)
colors = {macro: mcolors.to_hex(colormap(i)) for i, macro in enumerate(macro_list)}

# === Step 6: Plot Layout with Bounding Boxes ===

fig, ax = plt.subplots(figsize=(10, 10))

for cluster in clusters:
    x, y = cluster_positions[cluster]
    macro = assigned_macro[cluster]
    ax.plot(x, y, 'o', label=cluster, color=colors[macro])
    ax.text(x + 1, y + 1, cluster, fontsize=7)

for macro in macros:
    if macro in macro_positions:
        x, y = macro_positions[macro]
        ax.plot(x, y, 's', markersize=8, color=colors[macro])
        ax.text(x + 2, y + 2, macro, fontsize=9, weight='bold')
        
        corner, width = cluster_bounding_boxes.get(macro, (None, None))
        if corner is not None:
            rect = plt.Rectangle(corner, width, width, linewidth=1, edgecolor=colors[macro], facecolor='none', linestyle='--')
            ax.add_patch(rect)

ax.set_xlim(0, 120)
ax.set_ylim(0, 120)
ax.set_title("Macro-Centric Cluster Assignment and Bounding Boxes")
plt.grid(True)
layout_plot_path = "./updated_macro_cluster_layout.png"
plt.savefig(layout_plot_path)

# === Step 7: Create Macro Density Heatmap ===

grid_size = 10
grid_resolution = 120 // grid_size
density_grid = np.zeros((grid_resolution, grid_resolution))

for pos in macro_positions.values():
    x_idx = min(int(pos[0] // grid_size), grid_resolution - 1)
    y_idx = min(int(pos[1] // grid_size), grid_resolution - 1)
    density_grid[y_idx, x_idx] += 1

df_density = pd.DataFrame(density_grid)

plt.figure(figsize=(8, 6))
sns.heatmap(df_density, cmap='YlOrRd', cbar=True, square=True)
plt.title("Macro Placement Heat Density Map")
plt.xlabel("Grid X")
plt.ylabel("Grid Y")

heatmap_path = "./updated_macro_density_heatmap.png"
plt.savefig(heatmap_path)

layout_plot_path, heatmap_path
