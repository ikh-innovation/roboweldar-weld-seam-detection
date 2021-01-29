import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def reconstruction_filter(point_cloud, filter_radius=0.8, negative_filter=-0.15):
    """apply filters for Roboweldar reconstruction"""

    filter1 = lambda pts: np.linalg.norm(pts,
                                         axis=1) < filter_radius  # points that have euclidian distance from 0 < filter_radius
    filter2 = lambda pts: pts[:, 2] > negative_filter  # points that have height larger than negative_filter

    idxs = filter1(np.asarray(point_cloud.points))
    point_cloud = index_pointcloud(point_cloud, idxs)
    idxs = filter2(np.asarray(point_cloud.points))
    point_cloud = index_pointcloud(point_cloud, idxs)
    assert point_cloud.has_points(), "Pointcloud is probably not aligned to robot world frame"
    return point_cloud


def index_pointcloud(pt, idxs):
    pointcloud = o3d.geometry.PointCloud()
    '''return (new) pointcloud indexed by a list of indices'''
    pointcloud.points = o3d.utility.Vector3dVector(np.asarray(pt.points)[idxs])
    if pt.has_colors():
        pointcloud.colors = o3d.utility.Vector3dVector(np.asarray(pt.colors)[idxs])
    if pt.has_normals():
        pointcloud.normals = o3d.utility.Vector3dVector(np.asarray(pt.normals)[idxs])
    return pointcloud


def simplify_mesh(mesh, point_num=15000, triangles_target=1000):

    pointcloud = mesh.sample_points_poisson_disk(point_num, use_triangle_normal=True)
    pointcloud.paint_uniform_color([1, 0, 0])
    pointcloud = reconstruction_filter(pointcloud, filter_radius=0.4, negative_filter=-0.15)
    pointcloud = remove_clusters(pointcloud)

    rec_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pointcloud, depth=10, scale=1)
    rec_mesh.compute_vertex_normals()
    rec_mesh = rec_mesh.simplify_quadric_decimation(triangles_target, boundary_weight=5.0)
    print(rec_mesh)

    return  rec_mesh


def remove_clusters(pointcloud):
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pointcloud.cluster_dbscan(eps=0.1, min_points=10, print_progress=True))

    pts = np.asarray(pointcloud.points)
    unique, counts = np.unique(labels, return_counts=True)
    largest_label = unique[np.argmax(counts)]
    clean_points_idxs = [labels == largest_label]

    return index_pointcloud(pointcloud, clean_points_idxs)

    #VIS
    # max_label = labels.max()
    # print(f"point cloud has {max_label + 1} clusters")
    # colors = plt.cm.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    # colors[labels < 0] = 0
    # pointcloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

if __name__ == '__main__':
    mesh = o3d.io.read_triangle_mesh("/home/thanos/Downloads/transformed_mesh/transformed_mesh.obj")
    rec_mesh = simplify_mesh(mesh)
    o3d.visualization.draw_geometries([rec_mesh])
