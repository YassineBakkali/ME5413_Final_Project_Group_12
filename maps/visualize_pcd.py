import open3d as o3d

# Load the PCD file
pcd = o3d.io.read_point_cloud("BEST_MAP_EVER.pcd")

# Create a visualizer object
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add the point cloud to the visualizer
vis.add_geometry(pcd)

# Set the point size
opt = vis.get_render_option()
opt.point_size = 0.1  # Adjust this value as needed

# Run the visualizer
vis.run()
vis.destroy_window()

