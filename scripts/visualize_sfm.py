#!/usr/bin/env python3
"""
GTSAM-tutorial/scripts/visualize_sfm.py

Visualize 3D Structure from Motion (SfM) results from a text file.
This script reads camera poses and 3D point coordinates and plots them in a 3D space.
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
import os

def load_sfm_data(filename):
    """
    Loads SfM data from a file.
    The file should be structured with 'CAMERAS' and 'POINTS' sections.
    """
    cameras = {}
    points = {}
    current_section = None

    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                if line.startswith('CAMERAS'):
                    current_section = 'cameras'
                    continue
                elif line.startswith('POINTS'):
                    current_section = 'points'
                    continue

                parts = line.split()
                if len(parts) < 4:
                    continue

                label = parts[0]
                try:
                    # Handle potential extra info in camera lines by taking first 3 floats
                    coords = np.array([float(p) for p in parts[1:4]])
                    if current_section == 'cameras':
                        cameras[label] = coords
                    elif current_section == 'points':
                        points[label] = coords
                except ValueError:
                    print(f"Warning: Could not parse line: {line}")
                    continue

    except FileNotFoundError:
        print(f"Error: The file '{filename}' was not found.")
        print("Please run the C++ example first to generate the result file.")
        sys.exit(1)
        
    return cameras, points

def plot_sfm(cameras, points, output_filename="sfm_visualization.png"):
    """
    Plots the cameras and 3D points in a 3D scatter plot.
    """
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # Plot cameras
    if cameras:
        cam_coords = np.array(list(cameras.values()))
        ax.scatter(cam_coords[:, 0], cam_coords[:, 1], cam_coords[:, 2], c='blue', marker='^', s=100, label='Cameras')
        for label, pos in cameras.items():
            ax.text(pos[0], pos[1], pos[2], f' {label}', color='blue', fontsize=9)

    # Plot points
    if points:
        point_coords = np.array(list(points.values()))
        ax.scatter(point_coords[:, 0], point_coords[:, 1], point_coords[:, 2], c='red', marker='o', s=50, label='3D Points')
        for label, pos in points.items():
            ax.text(pos[0], pos[1], pos[2], f' {label}', color='red', fontsize=9)

    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('Structure from Motion (SfM) Visualization')
    ax.legend()
    ax.grid(True)

    # Auto-scaling axes to keep aspect ratio equal
    all_coords = []
    if cameras: all_coords.extend(cameras.values())
    if points: all_coords.extend(points.values())

    if all_coords:
        all_coords = np.array(all_coords)
        max_range = np.array([all_coords[:, 0].max()-all_coords[:, 0].min(), 
                              all_coords[:, 1].max()-all_coords[:, 1].min(), 
                              all_coords[:, 2].max()-all_coords[:, 2].min()]).max() / 2.0

        if max_range > 0:
            mid_x = (all_coords[:, 0].max()+all_coords[:, 0].min()) * 0.5
            mid_y = (all_coords[:, 1].max()+all_coords[:, 1].min()) * 0.5
            mid_z = (all_coords[:, 2].max()+all_coords[:, 2].min()) * 0.5
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)


    plt.savefig(output_filename)
    print(f"Visualization saved to '{output_filename}'")
    plt.show()

if __name__ == "__main__":
    # The script is in GTSAM-tutorial/scripts, so the result file is at ../result/sfm_result.txt
    script_dir = os.path.dirname(os.path.realpath(__file__))
    
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        # Default path relative to the project root
        filepath = os.path.join(script_dir, '..', 'result', 'sfm_result.txt')
    
    output_image_path = os.path.join(script_dir, '..', 'result', 'sfm_visualization.png')

    cameras, points = load_sfm_data(filepath)
    
    if not cameras and not points:
        print("No valid camera or point data found to visualize. Exiting.")
        sys.exit(0)

    plot_sfm(cameras, points, output_image_path)
