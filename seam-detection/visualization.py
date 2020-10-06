import open3d as o3d
import numpy as np
import argparse
import os


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

    # filename, file_extension = os.path.splitext(config.pc_path)
    # if file_extension == '.npy':
    #     pt = np.load(config.pc_path) # K,8
    #     print(pt,config.pc_path)
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(pt)
    # elif file_extension == '.npz':
    #
    #     pt = np.load(config.pc_path)['point_votes']  # K,8
    #
    #     pcd1 = o3d.geometry.PointCloud()
    #     pcd1.points = o3d.utility.Vector3dVector(pt[0:3])
    #     pcds.append(pcd1)
    #
    #     pcd2 = o3d.geometry.PointCloud()
    #     pcd2.points = o3d.utility.Vector3dVector(pt[3:6])
    #     pcds.append(pcd2)
    #
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(pt[6:9])
    #
    # else:
    pcd = o3d.io.read_triangle_mesh(config.pc_path)
    # pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd.compute_vertex_normals()
    print(np.asarray(pcd.triangles))

    pcds.append(pcd)

    #recolor white voxels if it is a pointcloud
    if hasattr(pcd, 'points'):
        colors = np.asarray(pcd.colors)
        white_idx =np.where((colors[:, 0] == 1) & (colors[:, 1] == 1) & (colors[:, 2] == 1))[0]
        colors[white_idx, :] = [0, 0, 0]
        pcd.colors = o3d.utility.Vector3dVector(colors)

    if config.gt_path is not None:
        ground_truth = o3d.io.read_point_cloud(config.gt_path)
        ground_truth.paint_uniform_color([1, 0, 0])
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
    'pc_path': "pointclouds/ArtificialPointClouds/TetrahedronMultiple.pcd",
}

if __name__ == '__main__':
    config = parse_args(default_config)
    main(config)
