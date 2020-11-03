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
# FLAGS.mesh_path = "/home/innovation/Projects/meshroom_workspace/MeshroomCache/Texturing/cd280aca667a07a1e47f1c78da56943a948afad7/texturedMesh.obj" #Π
# FLAGS.mesh_path = "/home/innovation/Projects/meshroom_workspace/baec704656c737bcb6e83b5f44fdf2a7ad420426/texturedMesh.obj" #Τ

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
    '''apply filters for Roboweldar reconstruction'''

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
    '''open3d type bounding boxes from Votenet prediction'''
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


def edge_intersection(edges, box_edge_indices, colorize=True):
    iter_indices = np.array([], dtype=int)
    colors = np.asarray(edges.colors)
    for subset in itertools.combinations(box_edge_indices, 2):
        intersection_indices = np.intersect1d(subset[0], subset[1])
        iter_indices = np.append(iter_indices, intersection_indices)

    iter_indices = np.unique(iter_indices)

    if colorize:
        colors[iter_indices] = np.array([0., 0., 1.])
        edges.colors = o3d.utility.Vector3dVector(colors)

    return index_pointcloud(edges, iter_indices)


def edges_within_bboxes(bboxes, edges_pt):
    '''returns non-empty bounding boxes and indeces of edges'''

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


def detect_trajectories(pointcloud: o3d.geometry.PointCloud, colorize=True, clusters_num=2) -> [o3d.geometry.Geometry3D]:
    import itertools
    from sklearn import mixture, linear_model, datasets
    from skimage.measure import LineModelND, ransac
    from matplotlib import pyplot as plt

    #CLUSTERING
    dataset = np.asarray(pointcloud.points)
    dpgmm = mixture.GaussianMixture(n_components=clusters_num, covariance_type='full').fit(dataset)
    predictions = dpgmm.predict(dataset)

    if colorize:
        colors = []
        for cl in predictions:
            colors.append([0, 0.4, 0]) if cl else colors.append([0, 0.7, 0])
        pointcloud.colors = o3d.utility.Vector3dVector(colors)

    #RANSAC LINE FITTING FOR EVERY CLUSTER
    lines_points = []
    correspondences = []
    for cl in range(clusters_num):
        cluster = dataset[predictions == cl]

        # robustly fit line only using inlier data with RANSAC algorithm
        model_robust, inliers = ransac(cluster, LineModelND, min_samples=2, residual_threshold=0.002, max_trials=100000)
        outliers = inliers == False

        # get the biggining and the end of the line to be predicted
        line_x = np.array([cluster[inliers][:, 0].min(), cluster[inliers][:, 0].max()])

        line_ransac = model_robust.predict(line_x)

        lines_points.append(line_ransac)
        correspondences.append([cl * 2, cl * 2 + 1])

    points = np.concatenate((*lines_points,), axis=0)
    colors = [[1, 0.8, 0] for i in range(len(correspondences))]
    trajectories = LineMesh(points, correspondences, colors, radius=0.002)
    trajectories_segments = trajectories.cylinder_segments

    return trajectories_segments


def main():
    try:
        mesh = o3d.io.read_triangle_mesh(FLAGS.mesh_path)
    except:
        print("Error reading triangle mesh")
        exit(-1)

    pcds = [mesh]

    # --------Pre-process----------
    point_cloud = mesh.sample_points_uniformly(number_of_points=int(FLAGS.num_point))
    point_cloud.paint_uniform_color([0.2, 0.8, 0.2])
    point_cloud = reconstruction_filter(point_cloud)

    # --------Detection----------
    predicted_pointcloud, predicted_edges_only = edge_detection(point_cloud)

    predictions = rbw_inference(FLAGS, np.asarray(point_cloud.points))  # VOTENET
    bboxes = parse_bounding_boxes(predictions)

    filtered_bboxes, filtered_bboxes_edges, filtered_bboxes_edges_indices = edges_within_bboxes(bboxes,
                                                                                                predicted_edges_only)
    pcds.extend(filtered_bboxes)

    intersection_points = edge_intersection(predicted_edges_only, filtered_bboxes_edges_indices)
    pcds.append(predicted_edges_only)
    # TODO detect edges in specific panels

    # --------Post-process----------

    traj = detect_trajectories(intersection_points)
    pcds.extend(traj)

    # path = path_finder(intersection_points)
    # pcds.append(path)

    pcds.append(intersection_points)

    o3d.visualization.draw_geometries(pcds)


if __name__ == '__main__':
    main()
