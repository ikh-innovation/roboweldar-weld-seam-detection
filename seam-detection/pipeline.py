import os
import sys
import numpy as np
import argparse
import importlib
import time
import open3d as o3d
import itertools

parser = argparse.ArgumentParser()
parser.add_argument('--mesh_path',
                    default="/home/innovation/Downloads/2020.09.29/part_2/transformed_mesh/transformed_mesh.obj",
                    type=str, help='Mesh absolute path.')
parser.add_argument('--filter_empty_boxes_num', type=int, default=1000,
                    help='Do not consider bounding boxes that contain less than this amount of points.')
FLAGS = parser.parse_args()

FLAGS.checkpoint_dir = 'log_panelnet/log_10-08-18:15'
FLAGS.cluster_sampling = 'vote_fps'
FLAGS.conf_thresh = 0.8
FLAGS.dataset = 'panelnet'
FLAGS.faster_eval = False
FLAGS.model = 'votenet'
FLAGS.nms_iou = 0.1
FLAGS.num_point = 300000
FLAGS.num_target = 1024
FLAGS.per_class_proposal = False
FLAGS.use_3d_nms = True
FLAGS.use_cls_nms = False
FLAGS.use_color = False
FLAGS.use_height = False
FLAGS.use_old_type_nms = False
FLAGS.vote_factor = 1

##TESTING
FLAGS.mesh_path = "/home/innovation/Downloads/2020.09.29/part_2/transformed_mesh/transformed_mesh.obj"
# FLAGS.mesh_path = "/home/innovation/Projects/meshroom_workspace/reconstruction_1/transformed_mesh/transformed_mesh.obj" #Π
# FLAGS.mesh_path = "/home/innovation/Projects/meshroom_workspace/reconstruction_2/transformed_mesh/transformed_mesh.obj" #Τ
# FLAGS.mesh_path = "/home/innovation/Projects/pytorch/votenet/panelnet/pc.ply"

# ------------Local Imports---------------
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(os.path.join(ROOT_DIR, 'utils'))
VOTENET_DIR = "/home/innovation/Projects/pytorch/votenet/"
sys.path.append(VOTENET_DIR)
sys.path.append(ROOT_DIR)
from inference import rbw_inference
from prediction import edge_detection
from lineMesh import LineMesh


def reconstruction_filter(point_cloud, filter_radius=0.5, negative_filter=-0.05):
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


def parse_bounding_boxes(predictions):
    """open3d type bounding boxes from Votenet prediction"""
    bboxes = []
    for box in predictions[0]:
        # transformations needed to translate votenet coordinates to NORMAL
        bounding_box = np.array(box[1])
        bounding_box[:, [0, 1, 2]] = bounding_box[:, [0, 2, 1]]
        bounding_box[:, 2] = bounding_box[:, 2] * -1
        box3d = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(bounding_box))
        box3d.color = np.array([1., 0.5, 0.])
        bboxes.append(box3d)
    return bboxes


def edge_intersection(edges: o3d.geometry.PointCloud, box_edge_indices: [[int]]) -> [[int]]:
    """ Gets a pointcloud and groups of indices that are inside a bounding box.
     It returns groups of indices that are common within 2 aforementioned boxes."""

    iter_indices_list = []
    colors = np.asarray(edges.colors)
    for subset in itertools.combinations(box_edge_indices, 2):
        intersection_indices = np.intersect1d(subset[0], subset[1])
        iter_indices_list.append(list(intersection_indices))

    # iter_indices_list = np.unique(iter_indices_list) #depricated
    # if colorize:
    #     colors[iter_indices_list] = np.array([0., 0., 1.])
    #     edges.colors = o3d.utility.Vector3dVector(colors)

    return iter_indices_list


def edges_within_bboxes(bboxes:[o3d.geometry.OrientedBoundingBox], edges_pt: o3d.geometry.PointCloud) -> ([o3d.geometry.OrientedBoundingBox], [o3d.geometry.PointCloud], [[int]]):
    """returns non-empty bounding boxes and indices of edges within"""

    filtered_bboxes = []
    filtered_bboxes_edges = []
    filtered_bboxes_edges_indices = []
    for bbox in bboxes:
        indices = bbox.get_point_indices_within_bounding_box(edges_pt.points)
        in_box_edges = index_pointcloud(edges_pt, indices)
        in_box_edges.paint_uniform_color([1., 0., 0.])

        if len(indices) < FLAGS.filter_empty_boxes_num: continue

        filtered_bboxes_edges_indices.append(indices)
        filtered_bboxes.append(bbox)
        filtered_bboxes_edges.append(in_box_edges)
    return filtered_bboxes, filtered_bboxes_edges, filtered_bboxes_edges_indices


# depricated
def path_finder(intersection_points):
    # distance, length
    graph = PointGraph(intersection_points)
    max_point = intersection_points.get_max_bound()
    min_point = intersection_points.get_min_bound()
    # o3d.geometry.PointCloud.compute_point_cloud_distance(intersection_points)
    path_points = BFS_SP(graph, 0, 30)
    path = make_3d_path(intersection_points, path_points)
    return path


# depricated
class PointGraph:
    def __init__(self, pointcloud: o3d.geometry.PointCloud):
        kd_tree = o3d.geometry.KDTreeFlann(pointcloud)
        self.edges = []
        self.dists = []
        for pt in pointcloud.points:
            neighbors = kd_tree.search_hybrid_vector_3d(pt, radius=0.01, max_nn=5)
            self.edges.append(neighbors[1][1:])
            self.dists.append(neighbors[2][1:])


# depricated
def BFS_SP(graph, start, goal):
    # Python implementation to find the shortest path in the graph using dictionaries
    # Function to find the shortest path between two nodes of a graph
    explored = []
    # Queue for traversing the graph in the BFS
    queue = [[start]]
    # If the desired node is reached
    if start == goal:
        print("Same Node")
        return
    # Loop to traverse the graph with the help of the queue
    while queue:
        path = queue.pop(0)
        node = path[-1]
        # Codition to check if the current node is not visited
        if node not in explored:
            neighbours = graph.edges[node]
            # Loop to iterate over the neighbours of the node
            for neighbour in neighbours:
                new_path = list(path)
                new_path.append(neighbour)
                queue.append(new_path)
                # Condition to check if the neighbour node is the goal
                if neighbour == goal:
                    print("Shortest path = ", *new_path)
                    return new_path
            explored.append(node)
        # Condition when the nodes are not connected
    print("So sorry, but a connecting path doesn't exist :(")
    return


# depricated
def make_3d_path(points, path):
    print(path)
    line = o3d.geometry.LineSet()

    line.points = o3d.utility.Vector3dVector(points.points)

    coorespondances = []
    for i in range(0, len(path) - 1):
        coorespondances.append([path[i], path[i + 1]])
    print(coorespondances)
    line.lines = o3d.utility.Vector2iVector(coorespondances)
    line.paint_uniform_color([0., 1., 0.])
    return line


def detect_trajectories(pointcloud: o3d.geometry.PointCloud, indices:[int], colorize=True, clusters_num=2, ransac_threshold = 0.002) -> ([o3d.geometry.Geometry3D], np.ndarray(shape=(4,4,2))):
    import itertools
    from sklearn import mixture, cluster
    from skimage.measure import LineModelND, ransac
    from matplotlib import pyplot as plt

    assert pointcloud.has_normals(), "pointcloud is missing normals"
    #CLUSTERING
    dataset = np.asarray(pointcloud.points)[indices]
    # cluster_algorithm = mixture.GaussianMixture(n_components=clusters_num, covariance_type='full', init_params='random', n_init=10, verbose=True).fit(dataset)
    # cluster_algorithm = mixture.BayesianGaussianMixture(n_components=clusters_num, covariance_type='full').fit(dataset)
    cluster_algorithm = cluster.OPTICS(min_cluster_size=100).fit(dataset)

    if hasattr(cluster_algorithm, 'labels_'):
        predictions = cluster_algorithm.labels_.astype(np.int)
    else:
        predictions = cluster_algorithm.predict(dataset)

    if colorize:
        cluster_colors = []
        colors_per_cluster = [np.random.choice(range(10), size=3)*0.1 for _ in range(-5, 50)]
        for cl in predictions:
            cluster_colors.append(colors_per_cluster[cl])

        edge_colors = np.asarray(pointcloud.colors)
        edge_colors[indices] = cluster_colors

        pointcloud.colors = o3d.utility.Vector3dVector(edge_colors)

    #RANSAC LINE FITTING FOR EVERY CLUSTER
    lines_points = []
    correspondences = []
    transformation_matrices = []
    for cl in range(clusters_num):
        cluster = dataset[predictions == cl]

        # robustly fit line only using inlier data with RANSAC algorithm
        model_robust, inliers = ransac(cluster, LineModelND, min_samples=2, residual_threshold=ransac_threshold, max_trials=10000)
        outliers = inliers == False

        #TRANSFORMATION MATRIX
        # get the beginning and the end of the line to be predicted
        line_x = np.array([cluster[inliers][:, 0].min(), cluster[inliers][:, 0].max()], dtype=np.float64)
        # predict the 2 points of the best fitting line
        line_ransac = model_robust.predict(line_x)
        # calculate the direction vector of the line
        line_direction = (line_ransac[1] - line_ransac[0])
        line_direction /= np.linalg.norm(line_direction)
        # calculate the normal of the line based on all the normals of the inlier points
        mean_normal = np.mean(np.asarray(pointcloud.normals)[indices][predictions == cl][inliers], axis=0, dtype=np.float64)
        mean_normal /= np.linalg.norm(mean_normal)
        #calculate the 3rd axis
        cross_product = np.cross(line_direction, mean_normal)

        rot_matrix = np.array([cross_product, line_direction, mean_normal], dtype=np.float64).T

        # test rotation matrix - results should be 1.
        # print("++++++++ROTATION MATRIX TEST+++++++++")
        # print(np.linalg.norm(line_direction))
        # print(np.linalg.norm(mean_normal))
        # print(np.linalg.norm(cross_product))
        # print(np.dot(rot_matrix, rot_matrix.T))
        # print(np.linalg.det(rot_matrix))

        # calculate the transformation matrix: translation + rotation
        trans_matrix = np.append(rot_matrix, np.array([[0,0,0]]), axis=0)
        line_begin = np.append(line_ransac[0].T, 1).reshape((4,1))
        line_end = np.append(line_ransac[1].T, 1).reshape((4,1))
        trans_matrix_begin = np.append(trans_matrix, line_begin, axis=1)
        trans_matrix_end = np.append(trans_matrix, line_end, axis=1)

        lines_points.append(line_ransac)
        correspondences.append([cl * 2, cl * 2 + 1])
        transformation_matrices.append(np.array([trans_matrix_begin, trans_matrix_end]))

    #VISUALIZATION
    # create a line mesh for each one
    yellow = [1, 0.8, 0]
    points = np.concatenate((*lines_points,), axis=0)
    cluster_colors = [yellow for i in range(len(correspondences))]
    trajectories = LineMesh(points, correspondences, cluster_colors, radius=ransac_threshold)
    trajectories_segments = trajectories.cylinder_segments

    # for every line, create and display the normal of it
    flat_trans_matrices = [item for sublist in transformation_matrices for item in sublist]
    for i, mtx in enumerate(flat_trans_matrices):
        # axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=ransac_threshold,cone_radius=ransac_threshold*1.3, cylinder_height=ransac_threshold*20, cone_height=ransac_threshold*3)
        arrow.paint_uniform_color(yellow)
        arrow.transform(mtx)
        trajectories_segments.append(arrow)

    return trajectories_segments, np.array(transformation_matrices)


def welding_paths_detection(mesh_path=FLAGS.mesh_path):
    try:
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        mesh.compute_vertex_normals()
    except:
        print("Error reading triangle mesh")
        exit(-1)

    pcds = [mesh]

    # --------Pre-process----------
    point_cloud = mesh.sample_points_uniformly(number_of_points=int(FLAGS.num_point))
    point_cloud.paint_uniform_color([0.2, 0.8, 0.2])
    point_cloud = reconstruction_filter(point_cloud)

    # --------Detection----------
    predicted_edges_only = edge_detection(point_cloud, k_n=60, thresh=0.02)

    predictions = rbw_inference(FLAGS, np.asarray(point_cloud.points))  # VOTENET
    bboxes = parse_bounding_boxes(predictions)

    filtered_bboxes, _, filtered_bboxes_edges_indices = edges_within_bboxes(bboxes, predicted_edges_only)
    pcds.extend(filtered_bboxes)

    intersection_point_indices_list = edge_intersection(predicted_edges_only, filtered_bboxes_edges_indices)
    # TODO detect edges in specific panels

    # --------Post-process----------

    welding_paths = []
    for indices in intersection_point_indices_list:
        if len(indices) == 0: continue
        trajectories_segments, transformation_matrices = detect_trajectories(predicted_edges_only, indices, colorize=True)
        pcds.extend(trajectories_segments)
        welding_paths.append(transformation_matrices)

    welding_paths = np.array(welding_paths)

    # path = path_finder(intersection_points)
    # pcds.append(path)

    pcds.append(predicted_edges_only)

    o3d.visualization.draw_geometries(pcds)

    """
    welding path numpy array format:
             welding lines xN
    starting/ending points x2
    transformation matrix 4x4
    """
    return welding_paths

if __name__ == '__main__':
    welding_paths_detection()