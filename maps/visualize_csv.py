import matplotlib.pyplot as plt
import numpy as np

# Load the 2D map data from the CSV file
map_2d = np.loadtxt('./2d_map.csv', delimiter=',')

# Plot the data
plt.scatter(map_2d[:, 0], map_2d[:, 1], s=1)  # s is the marker size
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.title('2D Map Visualization')
plt.axis('equal')  # Ensure equal scaling for both axes
plt.grid(True)
plt.show()
