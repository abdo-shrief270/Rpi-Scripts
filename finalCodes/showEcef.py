import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_earth_with_ecef(x, y, z):
    """
    Plot Earth as a sphere and a point for the given ECEF coordinates.
    Args:
        x, y, z: ECEF coordinates in meters.
    """
    # Earth's radius (mean radius in meters)
    R = 6371000

    # Create a sphere for Earth
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    xs = R * np.outer(np.cos(u), np.sin(v))
    ys = R * np.outer(np.sin(u), np.sin(v))
    zs = R * np.outer(np.ones_like(u), np.cos(v))

    # Create the figure
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the Earth
    ax.plot_surface(xs, ys, zs, rstride=5, cstride=5, color='blue', alpha=0.6, edgecolor='k')

    # Plot the ECEF point
    ax.scatter(x, y, z, color='red', s=100, label='ECEF Location')
    
    # Set axis labels
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Z (meters)")

    # Set limits
    max_val = R * 1.2  # Slightly larger than Earth's radius
    ax.set_xlim([-max_val, max_val])
    ax.set_ylim([-max_val, max_val])
    ax.set_zlim([-max_val, max_val])

    ax.legend()
    plt.title("Earth with ECEF Location")
    plt.show()

# Example ECEF coordinates
x = 4709455.889534627
y = 2857820.4046255923
z = 3203992.7391255177

# Call the function to plot
plot_earth_with_ecef(x, y, z)
