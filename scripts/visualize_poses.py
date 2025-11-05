#!/usr/bin/env python3
"""
Visualize GTSAM PoseSLAM results
"""
import matplotlib.pyplot as plt
import numpy as np

def load_poses(filename):
    """Load poses from file"""
    data = np.loadtxt(filename)
    keys = data[:, 0].astype(int)
    x = data[:, 1]
    y = data[:, 2]
    theta = data[:, 3]
    return keys, x, y, theta

def plot_poses(keys, x, y, theta, color, label, alpha=1.0):
    """Plot poses with arrows showing orientation"""
    plt.plot(x, y, 'o-', color=color, label=label, alpha=alpha, markersize=8, linewidth=2)
    
    # Draw orientation arrows
    arrow_length = 0.3
    for i, (xi, yi, ti) in enumerate(zip(x, y, theta)):
        dx = arrow_length * np.cos(ti)
        dy = arrow_length * np.sin(ti)
        plt.arrow(xi, yi, dx, dy, head_width=0.15, head_length=0.1, 
                 fc=color, ec=color, alpha=alpha)
        plt.text(xi + 0.15, yi + 0.15, f'x{keys[i]}', fontsize=10, 
                color=color, fontweight='bold')

def main():
    # Load initial and optimized poses
    try:
        keys_init, x_init, y_init, theta_init = load_poses('./result/poses_initial.txt')
        keys_opt, x_opt, y_opt, theta_opt = load_poses('./result/poses_optimized.txt')
    except FileNotFoundError:
        print("Error: Could not find pose files.")
        print("Please run ./gtsam_factor_graph first to generate the data.")
        return
    
    # Create figure
    plt.figure(figsize=(12, 10))
    
    # Plot initial estimate
    plot_poses(keys_init, x_init, y_init, theta_init, 'green', 
              'Initial Estimate', alpha=0.5)
    
    # Plot optimized result
    plot_poses(keys_opt, x_opt, y_opt, theta_opt, 'blue', 
              'Optimized Result', alpha=1.0)
    
    plt.xlabel('X (meters)', fontsize=12)
    plt.ylabel('Y (meters)', fontsize=12)
    plt.title('PoseSLAM: Initial Estimate vs Optimized Result', fontsize=14, fontweight='bold')
    plt.legend(fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    
    # Save figure
    plt.savefig('poseslam_result.png', dpi=150, bbox_inches='tight')
    print("Saved visualization to poseslam_result.png")
    
    # Show plot
    plt.show()

if __name__ == '__main__':
    main()
