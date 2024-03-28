import pcl
import numpy as np
import matplotlib.pyplot as plt

def load_point_cloud(file_path):
    return pcl.load(file_path)

def segment_ground(pcl_cloud):
    seg = pcl_cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PERPENDICULAR_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.45)  # Adjust this threshold for your application
    inliers, _ = seg.segment()
    return inliers

def segment_ground_passthrough(pcl_cloud, axis='z', min_axis_value=-1.5, max_axis_value=-0.5):
    """
    Segments the ground from a point cloud using a passthrough filter on a specified axis.

    Parameters:
    - pcl_cloud: The input point cloud data.
    - axis: Axis along which to apply the passthrough filter ('x', 'y', or 'z'). Default is 'z'.
    - min_axis_value: Minimum allowable value along the specified axis. Points with values less than this
                      will be filtered out. Default is -1.5.
    - max_axis_value: Maximum allowable value along the specified axis. Points with values greater than this
                      will be filtered out. Default is -0.5.

    Returns:
    - Filtered point cloud containing the points that are considered part of the ground.
    """
    
    # Create a PassThrough filter object.
    pass_through = pcl_cloud.make_passthrough_filter()
    
    # Set the filter axis.
    pass_through.set_filter_field_name(axis)
    
    # Set the allowable range along the axis.
    pass_through.set_filter_limits(min_axis_value, max_axis_value)
    
    # Apply the filter and return the filtered point cloud.
    return pass_through.filter()

def remove_ground(pcl_cloud, inliers):
    return pcl_cloud.extract(inliers, negative=True)

def project_to_2d(pcl_cloud):
    # This example assumes the point cloud is already aligned with the ground plane.
    # You may need to apply a transformation if this is not the case.
    return np.array([[point[0], point[1]] for point in pcl_cloud])

def save_map(map_2d, file_path):
    np.savetxt(file_path, map_2d, delimiter=',')

if __name__ == '__main__':
    # Load the point cloud
    pcl_cloud = load_point_cloud('BEST_MAP_EVER.pcd')

    # Segment and remove the ground
    ground_inliers = segment_ground(pcl_cloud)
    obstacles_cloud = remove_ground(pcl_cloud, ground_inliers)
    obstacles_cloud = segment_ground_passthrough(pcl_cloud, axis='z', min_axis_value=-0.188, max_axis_value=10)
    # Project the obstacles to 2D
    map_2d = project_to_2d(obstacles_cloud)
    
    # Plot the data
    plt.scatter(map_2d[:, 0], map_2d[:, 1], s=0.1)  # s is the marker size
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.title('2D Map Visualization')
    plt.axis('equal')  # Ensure equal scaling for both axes
    plt.grid(True)
    plt.show()


    # Save the 2D map
    save_map(map_2d, './2d_map.csv')
