import open3d as o3d
import argparse
import os

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--mesh', help='mesh full path to convert')
    parser.add_argument('--num_point', default=300000, help='point sampling number [300000]')
    mesh_path = parser.parse_args().mesh
    num_point = parser.parse_args().num_point

    file_path = os.path.splitext(mesh_path)[0]

    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.compute_vertex_normals()

    pcd = mesh.sample_points_uniformly(number_of_points=int(num_point), use_triangle_normal=True)

    o3d.io.write_point_cloud(file_path + ".pcd", pcd)

