import open3d as o3d
import numpy as np
import os

#HARDCODED FULL PATH incase
filename = r"C:\Users\tripa\OneDrive\Pictures\Desktop\2dx3project\pythoncode\tof_radar.xyz"
POINTS_PER_SCAN = 32
Z_SCALE_FACTOR = 20  # exaggerating Z spacing to visualize layers better

# DEBUG: Checking file path and print 
print("Looking for:", os.path.abspath(filename))
print("File exists:", os.path.exists(filename))

#LOADING DATA
try:
    points = np.loadtxt(filename)
    if points.ndim != 2 or points.shape[1] != 3: #must be 2d (rowsx columns) and musct have exactly 3 columns x y z
        raise ValueError("File must contain rows of 3D coordinates.")
except Exception as e:
    print(f"Error loading file: {e}")
    exit()

#ADJUSTING  Z SPACING
points[:, 2] *= Z_SCALE_FACTOR #multiplies only the x column by 20 to viuslaly spread the layers apart so the model does not look to squished 

num_points = points.shape[0] #divides by points per scan to find how many scan layers there are.... for example 3 scans * 32 points = 96 points 
num_scans = num_points // POINTS_PER_SCAN
print(f"Loaded {num_points} points across {num_scans} scans")

# CREATING POINT CLOUD 
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.paint_uniform_color([0.1, 0.8, 0.1])  # green points!

# CREATING THE  CONNECTION LINES
lines = []
colors = []

#vertical lines between matching points across scans
for scan_idx in range(num_scans - 1): #loop thru each scan layer execept the last one 
    for pt_idx in range(POINTS_PER_SCAN): #for each layer loop thru all the points in that scan
        start = scan_idx * POINTS_PER_SCAN + pt_idx #starting at one layer and one pint
        end = (scan_idx + 1) * POINTS_PER_SCAN + pt_idx #ending at next layer same point

        lines.append([start, end]) #adds the start, end pair to the lines list and tells open3d to ddraw a line from point 
        colors.append([1, 0, 0])  # red

# Blue horizontal lines within each scan circle
for scan_idx in range(num_scans): #loops thru every scan layer
    base = scan_idx * POINTS_PER_SCAN #base is the index of the first point in thisscan layer
    for pt_idx in range(POINTS_PER_SCAN): #loops over all 32 points in this scan
        start = base + pt_idx #connects point i to point i + 1
        end = base + (pt_idx + 1) % POINTS_PER_SCAN
        lines.append([start, end])
        colors.append([0, 0, 1])  # blue

# DRAWING THE MODELL
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector(colors)
 #Launch time 
print("Launching Open3D visualizer...")
o3d.visualization.draw_geometries([pcd, line_set])
