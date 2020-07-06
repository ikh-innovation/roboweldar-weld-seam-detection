import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import copy
import argparse
from pyntcloud import PyntCloud
import pymesh
import os


def mesh2pointcloud(mesh_path: str, point_num: int) -> o3d.geometry.PointCloud:
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    # mesh.scale(0.01, center=(0, 0, 0))
    # pointcloud = mesh.sample_points_poisson_disk(point_num)
    pointcloud = mesh.sample_points_uniformly(point_num)

    return pointcloud


def edge_detection(pcd: o3d.geometry.PointCloud, k_n=50, thresh=0.03) -> (o3d.geometry.PointCloud, o3d.geometry.PointCloud):

    pcd = PyntCloud.from_instance("open3d", pcd)

    pcd_np = np.zeros((len(pcd.points), 6))

    # find neighbors
    kdtree_id = pcd.add_structure("kdtree")
    k_neighbors = pcd.get_neighbors(k=k_n, kdtree=kdtree_id)

    # calculate eigenvalues
    ev = pcd.add_scalar_field("eigen_values", k_neighbors=k_neighbors)

    x = pcd.points['x'].values
    y = pcd.points['y'].values
    z = pcd.points['z'].values

    e1 = pcd.points['e3(' + str(k_n + 1) + ')'].values
    e2 = pcd.points['e2(' + str(k_n + 1) + ')'].values
    e3 = pcd.points['e1(' + str(k_n + 1) + ')'].values

    sum_eg = np.add(np.add(e1, e2), e3)
    sigma = np.divide(e1, sum_eg)

    edges = sigma > thresh

    points = np.transpose([x, y, z])

    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector(points)

    base_color = [[0.1, 0.1, 0.1]]
    red = [1, 0, 0]
    colors = np.repeat(base_color, len(edges), axis=0)
    colors[edges == True] = red
    pointcloud.colors = o3d.utility.Vector3dVector(colors)

    edges_pointcloud = o3d.geometry.PointCloud()
    edges_pointcloud.points = o3d.utility.Vector3dVector(points[sigma > thresh])
    edges_pointcloud.paint_uniform_color(red)

    return pointcloud, edges_pointcloud

class Config:
    def __init__(self,
                 pc_path: str
                 ):

        self.pc_path = pc_path



def main(config: Config):
    #read
    pcd_path = config.pc_path
    pcd = o3d.io.read_point_cloud(pcd_path)
    #if a pointcloud cannot be read try to read a mesh
    if len(pcd.points) == 0:
        pcd = mesh2pointcloud(pcd_path, 1000000)

    print(pcd)
    # mesh = o3d.io.read_triangle_mesh("ArtificialPointClouds/welding_area.obj")

    #predict
    predicted_pointcloud, predicted_edges_only = edge_detection(pcd)

    #visualize
    o3d.visualization.draw_geometries([predicted_pointcloud])

    #save
    output_dir = "./detected_edges/"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    o3d.io.write_point_cloud(output_dir + 'predicted_pointcloud.ply', predicted_pointcloud)
    o3d.io.write_point_cloud(output_dir + 'predicted_edges_only.ply', predicted_edges_only)


def parse_args(default_config: dict) -> Config:
    # -----------------ARGUMENTS-----------------#
    parser = argparse.ArgumentParser()
    parser.add_argument('--pc_path', dest='pc_path', default=default_config['pc_path'], type=str,
                        help='Path of the Point cloud to be visualized')



    args = parser.parse_args()

    return Config(
        pc_path=args.pc_path
    )

# ************************************************************************
default_config = {
    'pc_path': "ArtificialPointClouds/TetrahedronMultiple.pcd",
}

if __name__ == '__main__':
    config = parse_args(default_config)
    main(config)
