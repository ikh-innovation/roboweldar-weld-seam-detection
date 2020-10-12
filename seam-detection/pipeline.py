import os
import sys
import numpy as np
import argparse
import importlib
import time
import open3d as o3d
import itertools

parser = argparse.ArgumentParser()
parser.add_argument('--mesh_path', default="/home/innovation/Downloads/2020.09.29/part_2/transformed_mesh/transformed_mesh.obj", type=str, help='Mesh absolute path.')
parser.add_argument('--filter_empty_boxes_num', type=int, default=500, help='Do not consider bounding boxes that contain less than this amount of points.')
FLAGS = parser.parse_args()

FLAGS.checkpoint_dir='log_panelnet/log_10-08-18:15'
FLAGS.cluster_sampling='vote_fps'
FLAGS.conf_thresh=0.8
FLAGS.dataset='panelnet'
FLAGS.faster_eval=False
FLAGS.model='votenet'
FLAGS.nms_iou=0.1
FLAGS.num_point=300000
FLAGS.num_target=1024
FLAGS.per_class_proposal=False
FLAGS.use_3d_nms=True
FLAGS.use_cls_nms=False
FLAGS.use_color=False
FLAGS.use_height=False
FLAGS.use_old_type_nms=False
FLAGS.vote_factor=1


#------------Local Imports---------------
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(os.path.join(ROOT_DIR, 'utils'))
VOTENET_DIR = "/home/innovation/Projects/pytorch/votenet/"
sys.path.append(VOTENET_DIR)
sys.path.append(ROOT_DIR)
from inference import rbw_inference
from prediction import edge_detection


def reconstruction_filter(point_cloud, filter_radius=0.5, negative_filter=-0.05):
    '''apply filters for Roboweldar reconstruction'''

    filter1 = lambda pts: np.linalg.norm(pts, axis=1) < filter_radius  # points that have euclidian distance from 0 < filter_radius
    filter2 = lambda pts: pts[:, 2] > negative_filter # points that have height larger than negative_filter

    idxs = filter1(np.asarray(point_cloud.points))
    point_cloud = index_pointcloud(point_cloud, idxs)
    idxs = filter2(np.asarray(point_cloud.points))
    point_cloud = index_pointcloud(point_cloud, idxs)
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
        #transformations needed to translate votenet coordinates to NORMAL
        bounding_box = np.array(box[1])
        bounding_box[:, [0, 1, 2]] = bounding_box[:, [0, 2, 1]]
        bounding_box[:,2] = bounding_box[:,2] * -1
        box3d = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(bounding_box))
        box3d.color = np.array([1., 0.5, 0.])
        bboxes.append(box3d)
    return bboxes

def edge_intersection(edges, box_edge_indices):
    # iter_indices = np.array([])
    colors = np.asarray(edges.colors)
    for subset in itertools.combinations(box_edge_indices, 2):
        intersection_indices = np.intersect1d(subset[0], subset[1])
        colors[intersection_indices] = np.array([0., 0., 1.])

    edges.colors = o3d.utility.Vector3dVector(colors)

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

def main():
    try:
        mesh = o3d.io.read_triangle_mesh(FLAGS.mesh_path)
    except:
        print("Error reading triangle mesh")
        exit(-1)

    pcds = [mesh]

    #--------Pre-process----------
    point_cloud = mesh.sample_points_uniformly(number_of_points=int(FLAGS.num_point), use_triangle_normal=True)
    point_cloud.paint_uniform_color([0.2,0.8,0.2])
    point_cloud = reconstruction_filter(point_cloud)

    #--------Detection----------
    predicted_pointcloud, predicted_edges_only = edge_detection(point_cloud)

    predictions = rbw_inference(FLAGS, np.asarray(point_cloud.points)) #VOTENET
    bboxes = parse_bounding_boxes(predictions)

    filtered_bboxes, filtered_bboxes_edges, filtered_bboxes_edges_indices = edges_within_bboxes(bboxes, predicted_edges_only)
    # pcds.extend(filtered_bboxes_edges)
    pcds.extend(filtered_bboxes)

    edge_intersection(predicted_edges_only, filtered_bboxes_edges_indices)
    pcds.append(predicted_edges_only)
    #TODO detect edges in specific panels

    #--------Post-process----------





    o3d.visualization.draw_geometries(pcds)


if __name__=='__main__':
    main()