import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

def load_csv_map(csv_filename):
    # Load the CSV file as a NumPy array
    map_2d = np.loadtxt(csv_filename, delimiter=',')
    return map_2d

def create_occupancy_grid(map_2d, resolution, width, height, origin):
    # Initialize the occupancy grid
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = resolution
    grid.info.width = width
    grid.info.height = height
    grid.info.origin = Pose()
    grid.info.origin.position.x = origin[1]
    grid.info.origin.position.y = origin[0]

    # Fill in the occupancy data
    grid_data = np.full(width * height, 0, dtype=int)  # Initialize with -1 (unknown)
    for x, y in map_2d:
        # Convert (x, y) to grid coordinates
        grid_x = int((x - origin[1]) / resolution)
        grid_y = int((y - origin[0]) / resolution)
        if 0 <= grid_x < width and 0 <= grid_y < height:
            index = grid_y * width + grid_x
            grid_data[index] = 100  # Mark as occupied

    grid.data = list(grid_data)
    return grid

def publish_map(csv_filename, resolution, width, height, origin):
    rospy.init_node('csv_map_publisher')
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10, latch=True)

    # Load and create the occupancy grid
    map_2d = load_csv_map(csv_filename)
    grid = create_occupancy_grid(map_2d, resolution, width, height, origin)

    # Publish the map
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        grid.header.stamp = rospy.Time.now()
        map_pub.publish(grid)
        rate.sleep()

if __name__ == '__main__':
    csv_filename = '../maps/2d_map.csv'
    resolution = 0.05  # 5 cm per cell
    width = 1504  # Number of cells in the X direction
    height = 2080  # Number of cells in the Y direction
    origin = (-20.000000, -20.0000)  # Origin of the map in meters

    publish_map(csv_filename, resolution, width, height, origin)
