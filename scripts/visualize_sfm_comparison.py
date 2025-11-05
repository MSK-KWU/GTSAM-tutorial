#!/usr/bin/env python3
"""
GTSAM-tutorial/scripts/visualize_sfm_comparison.py

Visualize before/after comparison of Structure from Motion (SfM) optimization.
Shows initial estimates and optimized results side by side with camera poses and 3D points.
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

def plot_single_view(ax, cameras, points, title, show_labels=True):
    """
    Plots cameras and 3D points on a given axis.
    """
    # Plot cameras
    if cameras:
        cam_coords = np.array(list(cameras.values()))
        ax.scatter(cam_coords[:, 0], cam_coords[:, 1], cam_coords[:, 2], 
                   c='blue', marker='^', s=150, label='Cameras', alpha=0.8, edgecolors='black', linewidths=1.5)
        
        if show_labels:
            for label, pos in cameras.items():
                ax.text(pos[0], pos[1], pos[2], f' {label}', color='blue', fontsize=8, weight='bold')

    # Plot points
    if points:
        point_coords = np.array(list(points.values()))
        ax.scatter(point_coords[:, 0], point_coords[:, 1], point_coords[:, 2], 
                   c='red', marker='o', s=80, label='3D Points', alpha=0.7, edgecolors='darkred', linewidths=0.5)
        
        if show_labels:
            for label, pos in points.items():
                ax.text(pos[0], pos[1], pos[2], f' {label}', color='red', fontsize=7)

    ax.set_xlabel('X Axis', fontsize=10, weight='bold')
    ax.set_ylabel('Y Axis', fontsize=10, weight='bold')
    ax.set_zlabel('Z Axis', fontsize=10, weight='bold')
    ax.set_title(title, fontsize=12, weight='bold', pad=10)
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # Set equal aspect ratio
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

def plot_comparison(cameras_initial, points_initial, cameras_optimized, points_optimized, output_filename):
    """
    Creates a side-by-side comparison plot of initial and optimized SfM results.
    """
    fig = plt.figure(figsize=(20, 9))
    
    # Initial estimates (left)
    ax1 = fig.add_subplot(121, projection='3d')
    plot_single_view(ax1, cameras_initial, points_initial, 'Before Optimization (Initial Estimates)', show_labels=True)
    
    # Optimized results (right)
    ax2 = fig.add_subplot(122, projection='3d')
    plot_single_view(ax2, cameras_optimized, points_optimized, 'After Optimization (Final Results)', show_labels=True)
    
    plt.tight_layout()
    plt.savefig(output_filename, dpi=150, bbox_inches='tight')
    print(f"Comparison visualization saved to '{output_filename}'")
    plt.show()

def plot_overlay(cameras_initial, points_initial, cameras_optimized, points_optimized, output_filename):
    """
    Creates an overlay plot showing both initial and optimized results in the same view.
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot initial estimates
    if cameras_initial:
        cam_coords_init = np.array(list(cameras_initial.values()))
        ax.scatter(cam_coords_init[:, 0], cam_coords_init[:, 1], cam_coords_init[:, 2], 
                   c='lightblue', marker='^', s=120, label='Cameras (Initial)', alpha=0.5, edgecolors='blue', linewidths=1)
    
    if points_initial:
        point_coords_init = np.array(list(points_initial.values()))
        ax.scatter(point_coords_init[:, 0], point_coords_init[:, 1], point_coords_init[:, 2], 
                   c='lightcoral', marker='o', s=60, label='Points (Initial)', alpha=0.5, edgecolors='red', linewidths=0.5)
    
    # Plot optimized results
    if cameras_optimized:
        cam_coords_opt = np.array(list(cameras_optimized.values()))
        ax.scatter(cam_coords_opt[:, 0], cam_coords_opt[:, 1], cam_coords_opt[:, 2], 
                   c='darkblue', marker='^', s=150, label='Cameras (Optimized)', alpha=0.9, edgecolors='black', linewidths=1.5)
    
    if points_optimized:
        point_coords_opt = np.array(list(points_optimized.values()))
        ax.scatter(point_coords_opt[:, 0], point_coords_opt[:, 1], point_coords_opt[:, 2], 
                   c='darkred', marker='o', s=80, label='Points (Optimized)', alpha=0.8, edgecolors='black', linewidths=0.5)
    
    # Draw lines connecting initial to optimized positions
    if cameras_initial and cameras_optimized:
        for label in cameras_initial.keys():
            if label in cameras_optimized:
                init_pos = cameras_initial[label]
                opt_pos = cameras_optimized[label]
                ax.plot([init_pos[0], opt_pos[0]], 
                       [init_pos[1], opt_pos[1]], 
                       [init_pos[2], opt_pos[2]], 
                       'b--', alpha=0.3, linewidth=0.8)
    
    if points_initial and points_optimized:
        for label in points_initial.keys():
            if label in points_optimized:
                init_pos = points_initial[label]
                opt_pos = points_optimized[label]
                ax.plot([init_pos[0], opt_pos[0]], 
                       [init_pos[1], opt_pos[1]], 
                       [init_pos[2], opt_pos[2]], 
                       'r--', alpha=0.3, linewidth=0.8)
    
    ax.set_xlabel('X Axis', fontsize=11, weight='bold')
    ax.set_ylabel('Y Axis', fontsize=11, weight='bold')
    ax.set_zlabel('Z Axis', fontsize=11, weight='bold')
    ax.set_title('SfM Optimization: Initial vs Optimized (Overlay)', fontsize=13, weight='bold', pad=15)
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_filename, dpi=150, bbox_inches='tight')
    print(f"Overlay visualization saved to '{output_filename}'")
    plt.show()

def print_statistics(cameras_initial, points_initial, cameras_optimized, points_optimized):
    """
    Prints statistics about the optimization.
    """
    print("\n" + "="*60)
    print("SfM Optimization Statistics")
    print("="*60)
    
    # Camera displacement
    if cameras_initial and cameras_optimized:
        cam_displacements = []
        for label in cameras_initial.keys():
            if label in cameras_optimized:
                init_pos = cameras_initial[label]
                opt_pos = cameras_optimized[label]
                displacement = np.linalg.norm(opt_pos - init_pos)
                cam_displacements.append(displacement)
        
        if cam_displacements:
            print(f"\nCamera Poses:")
            print(f"  Number of cameras: {len(cameras_initial)}")
            print(f"  Average displacement: {np.mean(cam_displacements):.4f}")
            print(f"  Max displacement: {np.max(cam_displacements):.4f}")
            print(f"  Min displacement: {np.min(cam_displacements):.4f}")
    
    # Point displacement
    if points_initial and points_optimized:
        point_displacements = []
        for label in points_initial.keys():
            if label in points_optimized:
                init_pos = points_initial[label]
                opt_pos = points_optimized[label]
                displacement = np.linalg.norm(opt_pos - init_pos)
                point_displacements.append(displacement)
        
        if point_displacements:
            print(f"\n3D Points:")
            print(f"  Number of points: {len(points_initial)}")
            print(f"  Average displacement: {np.mean(point_displacements):.4f}")
            print(f"  Max displacement: {np.max(point_displacements):.4f}")
            print(f"  Min displacement: {np.min(point_displacements):.4f}")
    
    print("="*60 + "\n")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.realpath(__file__))
    result_dir = os.path.join(script_dir, '..', 'result')
    
    # File paths
    initial_file = os.path.join(result_dir, 'sfm_result_initial.txt')
    optimized_file = os.path.join(result_dir, 'sfm_result_optimized.txt')
    
    comparison_output = os.path.join(result_dir, 'sfm_comparison.png')
    overlay_output = os.path.join(result_dir, 'sfm_overlay.png')
    
    # Load data
    print("Loading SfM data...")
    cameras_initial, points_initial = load_sfm_data(initial_file)
    cameras_optimized, points_optimized = load_sfm_data(optimized_file)
    
    if not (cameras_initial or points_initial) or not (cameras_optimized or points_optimized):
        print("No valid data found to visualize. Exiting.")
        sys.exit(0)
    
    # Print statistics
    print_statistics(cameras_initial, points_initial, cameras_optimized, points_optimized)
    
    # Create visualizations
    print("\nGenerating visualizations...")
    plot_comparison(cameras_initial, points_initial, cameras_optimized, points_optimized, comparison_output)
    plot_overlay(cameras_initial, points_initial, cameras_optimized, points_optimized, overlay_output)
    
    print("\n✓ Visualization complete!")
