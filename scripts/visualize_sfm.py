#!/usr/bin/env python3
"""
Visualize GTSAM Structure from Motion (SfM) results in 3D
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def load_sfm_results(filename):
    """Load SfM results from file"""
    cameras = {}
    points = {}
    
    with open(filename, 'r') as f:
        section = None
        for line in f:
            line = line.strip()
            if line == 'CAMERAS':
                section = 'cameras'
                continue
            elif line == 'POINTS':
                section = 'points'
                continue
            
            if section == 'cameras' and line:
                parts = line.split()
                name = parts[0]
                pos = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                cameras[name] = pos
            elif section == 'points' and line:
                parts = line.split()
                name = parts[0]
                pos = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                points[name] = pos
    
    return cameras, points

def plot_camera(ax, position, label, size=0.5):
    """Plot a camera as a pyramid"""
    # Camera center
    ax.scatter(*position, c='blue', marker='o', s=100, label=label if label else None)
    
    # Camera coordinate frame
    ax.quiver(position[0], position[1], position[2], 
              size, 0, 0, color='r', arrow_length_ratio=0.3, linewidth=2)
    ax.quiver(position[0], position[1], position[2], 
              0, size, 0, color='g', arrow_length_ratio=0.3, linewidth=2)
    ax.quiver(position[0], position[1], position[2], 
              0, 0, size, color='b', arrow_length_ratio=0.3, linewidth=2)
    
    # Add label
    ax.text(position[0], position[1], position[2], f'  {label}', 
            fontsize=12, fontweight='bold', color='blue')

def plot_point(ax, position, label):
    """Plot a 3D point"""
    ax.scatter(*position, c='red', marker='o', s=50)
    ax.text(position[0], position[1], position[2], f'  {label}', 
            fontsize=10, color='red')

def main():
    # Load results
    try:
        cameras, points = load_sfm_results('result/sfm_result.txt')
    except FileNotFoundError:
        print("Error: Could not find result/sfm_result.txt")
        print("Please run ./sfm_example first to generate the data.")
        return
    
    # Create 3D plot
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot cameras
    for name, pos in cameras.items():
        plot_camera(ax, pos, name, size=0.5)
    
    # Plot 3D points
    for name, pos in points.items():
        plot_point(ax, pos, name)
    
    # Draw lines from cameras to points (viewing rays)
    for cam_name, cam_pos in cameras.items():
        for point_name, point_pos in points.items():
            ax.plot([cam_pos[0], point_pos[0]], 
                   [cam_pos[1], point_pos[1]], 
                   [cam_pos[2], point_pos[2]], 
                   'gray', alpha=0.2, linewidth=0.5)
    
    # Set labels and title
    ax.set_xlabel('X', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z', fontsize=12, fontweight='bold')
    ax.set_title('Structure from Motion (SfM) Result\n3D Points and Camera Poses', 
                 fontsize=14, fontweight='bold')
    
    # Set equal aspect ratio
    max_range = 0
    for pos in list(cameras.values()) + list(points.values()):
        max_range = max(max_range, np.abs(pos).max())
    
    ax.set_xlim([-max_range*1.2, max_range*1.2])
    ax.set_ylim([-max_range*1.2, max_range*1.2])
    ax.set_zlim([-max_range*1.2, max_range*1.2])
    
    # Add legend
    ax.legend(['Cameras', '3D Points'], fontsize=12)
    
    # Add grid
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save figure
    plt.savefig('sfm_result_3d.png', dpi=150, bbox_inches='tight')
    print("Saved visualization to sfm_result_3d.png")
    
    # Show plot
    plt.show()

if __name__ == '__main__':
    main()
