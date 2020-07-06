import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import copy
import argparse
import pymesh


def mesh2pointcloud(mesh_path: str, point_num: int) -> o3d.geometry.PointCloud:
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    pointcloud = mesh.sample_points_uniformly(point_num)
    # pointcloud = mesh.sample_points_uniformly(point_num).scale(0.25)
    pointcloud.normals = mesh.vertex_normals
    pointcloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

    return pointcloud



class Config:
    def __init__(self,
                 pc_path: str,
                 gt_path: str
                 ):

        self.pc_path = pc_path
        self.gt_path = gt_path



def main(config: Config):

    pcds = []

    pointcloud = o3d.io.read_point_cloud(config.pc_path)
    pcds.append(pointcloud)

    #recolor if pc is white
    colors = np.asarray(pointcloud.colors)
    white_idx =np.where((colors[:, 0] == 1) & (colors[:, 1] == 1) & (colors[:, 2] == 1))[0]
    colors[white_idx, :] = [0, 0, 0]
    pointcloud.colors = o3d.utility.Vector3dVector(colors)

    if config.gt_path is not None:
        ground_truth = o3d.io.read_point_cloud(config.gt_path)
        ground_truth.paint_uniform_color([0, 1, 0])
        pcds.append(ground_truth)

    o3d.visualization.draw_geometries(pcds)




def parse_args(default_config: dict) -> Config:
    # -----------------ARGUMENTS-----------------#
    parser = argparse.ArgumentParser()
    parser.add_argument('--pc_path', dest='pc_path', default=default_config['pc_path'], type=str,
                        help='Path of the Point cloud to be visualized')
    parser.add_argument('--gt_path', dest='gt_path', type=str, nargs='?',
                        help='Path of the ground Truth of the Point Cloud')


    args = parser.parse_args()

    return Config(
        pc_path=args.pc_path,
        gt_path=args.gt_path
    )

# ************************************************************************
default_config = {
    'pc_path': "ArtificialPointClouds/TetrahedronMultiple.pcd",
}

if __name__ == '__main__':
    config = parse_args(default_config)
    main(config)
