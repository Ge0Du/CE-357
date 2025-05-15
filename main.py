import os
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import seaborn as sns
import pandas as pd
import random
import math
from shapely.geometry import box
from collections import defaultdict
import matplotlib.cm as cm
# === PARAMETERS ===
NUM_MACROS = 10
NUM_STANDARD_CELLS = 10000
MACRO_SIZE_RANGE = (10, 30)
CLUSTERS = 200
CHIP_SIZE = 85
GRID_SIZE = 1
GRID_DIM = CHIP_SIZE // GRID_SIZE
NUM_CLUSTER_CONNECTIONS = 3

# === OUTPUT DIRECTORY ===
output_dir = "./Algo5(Dual O)"
os.makedirs(output_dir, exist_ok=True)

np.random.seed(42)
random.seed(42)

# === MACROS & STANDARD CELLS ===
macros = [f'M{i+1}' for i in range(NUM_MACROS)]
macro_sizes = {m: (random.randint(*MACRO_SIZE_RANGE), random.randint(*MACRO_SIZE_RANGE)) for m in macros}
standard_cells = [f'C{i+1}' for i in range(NUM_STANDARD_CELLS)]

# === STANDARD CELL CONNECTIVITY GRAPH ===
cell_graph = nx.Graph()
cell_graph.add_nodes_from(standard_cells)
for _ in range(NUM_STANDARD_CELLS * 2):
    u, v = random.sample(standard_cells, 2)
    cell_graph.add_edge(u, v, weight=random.randint(1, 10))

# === CLUSTERING ===
clusters = defaultdict(list)
for i, c in enumerate(standard_cells):
    clusters[i % CLUSTERS].append(c)
cluster_ids = list(clusters.keys())

# === MACRO ↔ CLUSTER ASSIGNMENTS ===
macro_to_clusters = defaultdict(set)
cluster_to_macros = defaultdict(set)
for cid in cluster_ids:
    connected_macros = random.sample(macros, k=random.randint(1, 3))
    for m in connected_macros:
        macro_to_clusters[m].add(cid)
        cluster_to_macros[cid].add(m)

# === CLUSTER CONNECTIVITY GRAPH (WITH MACRO-AFFINITY EDGES) ===
cluster_graph = nx.Graph()
cluster_graph.add_nodes_from(cluster_ids)

# Add connectivity edges
for cid in cluster_ids:
    connections = random.sample([c for c in cluster_ids if c != cid], k=NUM_CLUSTER_CONNECTIONS)
    for other_cid in connections:
        cluster_graph.add_edge(cid, other_cid, weight=random.randint(1, 10))

# Add macro-affinity edges
for macro, clist in macro_to_clusters.items():
    clist = list(clist)
    for i in range(len(clist)):
        for j in range(i + 1, len(clist)):
            u, v = clist[i], clist[j]
            if not cluster_graph.has_edge(u, v):
                cluster_graph.add_edge(u, v, weight=1)
            else:
                cluster_graph[u][v]['weight'] += 1

# === FORCE-DIRECTED CLUSTER PLACEMENT ===
raw_pos = nx.spring_layout(cluster_graph, weight='weight', seed=42)
positions_array = np.array(list(raw_pos.values()))
min_vals = positions_array.min(axis=0)
max_vals = positions_array.max(axis=0)
normalized_positions = {
    node: (pos[0] - min_vals[0], pos[1] - min_vals[1])
    for node, pos in raw_pos.items()
}
range_vals = max_vals - min_vals
cluster_positions = {
    node: (norm[0] / range_vals[0] * CHIP_SIZE, norm[1] / range_vals[1] * CHIP_SIZE)
    for node, norm in normalized_positions.items()
}

# === MACRO PLACEMENT ===
def is_overlapping(new_box, existing_boxes):
    return any(new_box.intersects(b) for b in existing_boxes)

def is_within_chip(b: box, chip_size: int) -> bool:
    minx, miny, maxx, maxy = b.bounds
    return 0 <= minx and maxx <= chip_size and 0 <= miny and maxy <= chip_size
def spiral_search(cx, cy, w, h, chip_size, step_frac=0.01, angle_steps=24):
    """
    Spiral search for macro placement with orientation consideration.
    Tries both (w, h) and (h, w) and picks the one that stays more compactly inside the chip.
    Returns (position, box, chosen_w, chosen_h) or None if failed.
    """
    max_radius = chip_size / 2
    step_r = chip_size * step_frac
    step_theta = 360 // angle_steps
    r = 0.0

    while r <= max_radius:
        for angle_deg in range(0, 360, step_theta):
            theta = math.radians(angle_deg)
            dx, dy = r * math.cos(theta), r * math.sin(theta)
            x, y = cx + dx, cy + dy

            candidates = [(w, h), (h, w)]
            best = None
            min_extent = float('inf')

            for test_w, test_h in candidates:
                b = box(x - test_w / 2, y - test_h / 2, x + test_w / 2, y + test_h / 2)
                if is_within_chip(b, chip_size) and not is_overlapping(b, macro_boxes):
                    extent = max(b.bounds[2], b.bounds[3])  # farthest x or y
                    if extent < min_extent:
                        best = (np.array([x, y]), b, test_w, test_h)
                        min_extent = extent

            if best:
                return best  # (position, box, chosen_w, chosen_h)

        r += step_r

    return None, None, None, None  # Failed

macro_positions = {}
macro_boxes = []
sorted_macros = sorted(macros, key=lambda m: len(macro_to_clusters[m]), reverse=True)

for macro in sorted_macros:
    related_clusters = macro_to_clusters[macro]
    xs = [cluster_positions[cid][0] for cid in related_clusters]
    ys = [cluster_positions[cid][1] for cid in related_clusters]
    cx, cy = np.mean(xs), np.mean(ys)
    w, h = macro_sizes[macro]
    pos, b, chosen_w, chosen_h = spiral_search(cx, cy, w, h, CHIP_SIZE)
    if pos is not None:
        macro_positions[macro] = pos
        macro_boxes.append(b)
        macro_sizes[macro] = (chosen_w, chosen_h)  # update size in case rotated
    else:
        print(f" Could not legally place macro {macro}. Assigning to (0,0)")
        macro_positions[macro] = np.array([0, 0])
        macro_sizes[macro] = (w, h)  # keep original size


# === VISUALIZATIONS ===

# Layout: Macros + Clusters + Connectivity
fig, ax = plt.subplots(figsize=(10, 10))
for cid, (x, y) in cluster_positions.items():
    ax.plot(x, y, 'o', color='lightblue', markersize=4)
for macro in macros:
    x, y = macro_positions[macro]
    w, h = macro_sizes[macro]
    ax.plot(x, y, 's', color='red', markersize=6)
    ax.add_patch(plt.Rectangle((x - w/2, y - h/2), w, h, linewidth=1, edgecolor='red', facecolor='none'))
    ax.text(x + 0.5, y + 0.5, macro, fontsize=7, fontweight='bold')
for macro in macros:
    x0, y0 = macro_positions[macro]
    for cid in macro_to_clusters[macro]:
        x1, y1 = cluster_positions[cid]
        ax.plot([x0, x1], [y0, y1], 'gray', linestyle='--', linewidth=0.8, alpha=0.6)
ax.set_xlim(0, CHIP_SIZE)
ax.set_ylim(0, CHIP_SIZE)
ax.set_title("Macro-to-Cluster Connectivity Layout")
ax.grid(True)
plt.savefig(f"{output_dir}/macro_cluster_connectivity_layout.png")

# Individual Macro Views
for macro in macros:
    fig, ax = plt.subplots(figsize=(6, 6))
    x0, y0 = macro_positions[macro]
    w, h = macro_sizes[macro]
    ax.plot(x0, y0, 's', color='red', markersize=6)
    ax.text(x0 + 0.5, y0 + 0.5, macro, fontsize=7, fontweight='bold')
    ax.add_patch(plt.Rectangle((x0 - w/2, y0 - h/2), w, h, linewidth=1, edgecolor='red', facecolor='none'))
    for cid in macro_to_clusters[macro]:
        x1, y1 = cluster_positions[cid]
        ax.plot(x1, y1, 'o', color='lightblue', markersize=4)
        ax.text(x1 + 0.5, y1 + 0.5, f'Cl{cid}', fontsize=6)
        ax.plot([x0, x1], [y0, y1], 'gray', linestyle='--', linewidth=0.8, alpha=0.6)
    ax.set_xlim(0, CHIP_SIZE)
    ax.set_ylim(0, CHIP_SIZE)
    ax.set_title(f"{macro} and Connected Clusters")
    ax.grid(True)
    plt.savefig(f"{output_dir}/{macro}_connectivity_view.png")


# Macro Density Heatmap
macro_density = np.zeros((GRID_DIM, GRID_DIM))
for macro, (x, y) in macro_positions.items():
    x_idx = min(int(x // GRID_SIZE), GRID_DIM - 1)
    y_idx = min(int(y // GRID_SIZE), GRID_DIM - 1)
    macro_density[y_idx, x_idx] += 1
plt.figure(figsize=(6, 5))
sns.heatmap(pd.DataFrame(macro_density), cmap='Reds', cbar=True, square=True)
plt.title("Macro Density Heatmap")
plt.xlabel("X Grid")
plt.ylabel("Y Grid")
macro_density_path = f"{output_dir}/macro_density_heatmap.png"
plt.savefig(macro_density_path)

# Cluster Density Heatmap
cluster_density = np.zeros((GRID_DIM, GRID_DIM))
for _, (x, y) in cluster_positions.items():
    x_idx = min(int(x // GRID_SIZE), GRID_DIM - 1)
    y_idx = min(int(y // GRID_SIZE), GRID_DIM - 1)
    cluster_density[y_idx, x_idx] += 1
plt.figure(figsize=(6, 5))
sns.heatmap(pd.DataFrame(cluster_density), cmap='Blues', cbar=True, square=True)
plt.title("Cluster Density Heatmap")
plt.xlabel("X Grid")
plt.ylabel("Y Grid")
cluster_density_path = f"{output_dir}/cluster_density_heatmap.png"
plt.savefig(cluster_density_path)

# Cluster Map Colored by Macro Families
macro_colors = {}
macro_list = list(macro_to_clusters.keys())
cmap = cm.get_cmap('tab10', len(macro_list))
for i, m in enumerate(macro_list):
    macro_colors[m] = cmap(i)

fig, ax = plt.subplots(figsize=(10, 10))
for macro, clusters in macro_to_clusters.items():
    color = macro_colors[macro]
    for cid in clusters:
        x, y = cluster_positions[cid]
        ax.plot(x, y, 'o', color=color, markersize=4)
        ax.text(x + 0.3, y + 0.3, f'{cid}', fontsize=5)

for macro in macros:
    x, y = macro_positions[macro]
    w, h = macro_sizes[macro]
    ax.plot(x, y, 's', color=macro_colors[macro], markersize=8)
    rect = plt.Rectangle((x - w/2, y - h/2), w, h, linewidth=1.5,
                         edgecolor=macro_colors[macro], facecolor='none')
    ax.add_patch(rect)
    ax.text(x + 0.5, y + 0.5, macro, fontsize=7, fontweight='bold')

ax.set_xlim(0, CHIP_SIZE)
ax.set_ylim(0, CHIP_SIZE)
ax.set_title("Cluster Map Colored by Macro Families")
ax.grid(True)
cluster_macro_colormap_path = f"{output_dir}/cluster_macro_family_colormap.png"
plt.savefig(cluster_macro_colormap_path)

macro_density_path, cluster_density_path, cluster_macro_colormap_path