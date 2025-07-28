import open3d as o3d

pcd = o3d.io.read_point_cloud("/home/ibrahim/Downloads/projected_map_2d.pcd")
o3d.visualization.draw_geometries([pcd])