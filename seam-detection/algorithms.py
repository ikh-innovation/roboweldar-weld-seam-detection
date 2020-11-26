import open3d as o3d
import numpy as np
import argparse
from pyntcloud import PyntCloud
import os
import copy

# o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)


def mesh2pointcloud(mesh_path: str, point_num: int) -> o3d.geometry.PointCloud:
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    # mesh.scale(0.01, center=(0, 0, 0))
    # pointcloud = mesh.sample_points_poisson_disk(point_num)
    pointcloud = mesh.sample_points_uniformly(point_num)
    pointcloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

    return pointcloud


def panel_detection(pcd: o3d.geometry.PointCloud):
    type_mean_size = {'panel': np.array([0.012, 0.30, 0.20]),
                           'hpanel': np.array([0.30, 0.30, 0.012])}

    box = o3d.geometry.TriangleMesh.create_box(0.012, 0.30, 0.20)
    box.compute_vertex_normals()

    box_pt = box.sample_points_uniformly(number_of_points=10000,  use_triangle_normal=True)


    source = pcd
    target = box_pt
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    # source.transform(trans_init)
    # draw_registration_result(source, target, np.identity(4))

    voxel_size = 0.005
    source_down, source_fpfh = preprocess_point_cloud(pcd, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(box_pt, voxel_size)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print(result_ransac)
    draw_registration_result(source_down, target_down, result_ransac.transformation)


def pointcloud_from_bounding_box(bbox: o3d.geometry.AxisAlignedBoundingBox, pt_num=5000) -> o3d.geometry.PointCloud:
    bpoints = bbox.get_box_points()
    bpointcloud = o3d.geometry.PointCloud()
    bpointcloud.points = bpoints
    hull, _ = bpointcloud.compute_convex_hull()
    pt = hull.sample_points_uniformly(pt_num)
    pt.estimate_normals()
    pt.paint_uniform_color([0.5, 0.5, 0.5])
    return pt


def panel_registration(pcd: o3d.geometry.PointCloud, bboxes: [o3d.geometry.AxisAlignedBoundingBox], pt_num: int):
    pointboxes = []
    for bbox in bboxes:
        pointbox = pointcloud_from_bounding_box(bbox, pt_num)
        # point to plane ICP
        current_transformation = np.identity(4)
        print("2. Point-to-plane ICP registration is applied on original point")
        print("   clouds to refine the alignment. Distance threshold 0.02.")
        result_icp = o3d.registration.registration_icp(pointbox, pcd, 0.02, current_transformation,
            o3d.registration.TransformationEstimationPointToPlane())
        pointboxes.append(pointbox)
        print(result_icp.transformation)
        bbox.rotate(result_icp.transformation[:3,:3], bbox.get_center())
        bbox.translate(result_icp.transformation[0:3,3])
        draw_registration_result(pointbox, pcd, result_icp.transformation)

    return pointboxes

def draw_registration_result(source, target, transformation):
    print("transformation", transformation)
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return pcd_down, pcd_fpfh

def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.1
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)

    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [ ], o3d.registration.RANSACConvergenceCriteria(4000000, 50000))

    # result = o3d.registration.registration_ransac_based_on_correspondence(
    #     source_down, target_down, o3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold), distance_threshold)

    return result


def edge_detection(pcd: o3d.geometry.PointCloud, k_n=50, thresh=0.03) -> o3d.geometry.PointCloud:
    """detects edges of a pointcloud and colors the respective points. It returns another pointcloud including only the edges"""

    pcd_pynt = PyntCloud.from_instance("open3d", pcd)

    pcd_np = np.zeros((len(pcd_pynt.points), 6))

    # find neighbors
    kdtree_id = pcd_pynt.add_structure("kdtree")
    k_neighbors = pcd_pynt.get_neighbors(k=k_n, kdtree=kdtree_id)

    # calculate eigenvalues #TODO: docstring
    ev = pcd_pynt.add_scalar_field("eigen_values", k_neighbors=k_neighbors)

    x = pcd_pynt.points['x'].values
    y = pcd_pynt.points['y'].values
    z = pcd_pynt.points['z'].values

    e1 = pcd_pynt.points['e3(' + str(k_n + 1) + ')'].values
    e2 = pcd_pynt.points['e2(' + str(k_n + 1) + ')'].values
    e3 = pcd_pynt.points['e1(' + str(k_n + 1) + ')'].values

    sum_eg = np.add(np.add(e1, e2), e3)
    sigma = np.divide(e1, sum_eg)

    edges = sigma > thresh

    points = np.transpose([x, y, z])

    pcd.points = o3d.utility.Vector3dVector(points)

    base_color = [[0.1, 0.1, 0.1]]
    red = [1, 1, 1]
    colors = np.repeat(base_color, len(edges), axis=0)
    colors[edges == True] = red
    pcd.colors = o3d.utility.Vector3dVector(colors)

    edges_pointcloud = o3d.geometry.PointCloud()
    edges_pointcloud.points = o3d.utility.Vector3dVector(points[sigma > thresh])
    edges_pointcloud.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[sigma > thresh])
    edges_pointcloud.paint_uniform_color(red)

    return edges_pointcloud

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

    panel_detection(pcd)
    exit()

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
    'pc_path': "/home/innovation/Downloads/2020.09.29/part_2/transformed_mesh/transformed_mesh.obj",
}

if __name__ == '__main__':
    config = parse_args(default_config)
    main(config)
