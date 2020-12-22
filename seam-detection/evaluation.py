import os
import numpy
import open3d as o3d
import json
import numpy as np
import itertools
from pipeline import welding_paths_detection, points_intersection
from trajectory_annotator import json2bboxes, json2trajectories, create_mesh_lines

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
EVAL_DIR = os.path.join(ROOT_DIR, 'welding_scenes_eval')

# sys.path.append(os.path.join(ROOT_DIR, 'utils'))

def trajectory_eval(gt, pred):
    return 0

def linear_segment_dist(seg1, seg2):
    metric = np.linalg.norm(seg1[0]-seg2[0])
    metric += np.linalg.norm(seg1[0]-seg2[1])
    metric += np.linalg.norm(seg1[1]-seg2[0])
    metric += np.linalg.norm(seg1[1]-seg2[1])
    return metric/2



def calculate_trajectories_distance(pred, gt):
    for pair in pred:

        for gt_pair in gt: return

def calculate_bboxes_iou(pt:o3d.geometry.PointCloud, pred:[o3d.geometry.OrientedBoundingBox], gt:[o3d.geometry.OrientedBoundingBox]):
    pred_ious = []
    for bbox_pred in pred:
        pred_indices = bbox_pred.get_point_indices_within_bounding_box(pt.points)
        top_inter_num = 0
        iou = 0
        for bbox_gt in gt:
            gt_indices = bbox_gt.get_point_indices_within_bounding_box(pt.points)
            inter_indices = points_intersection(pt, [pred_indices, gt_indices])
            if len(inter_indices[0]) > top_inter_num:
                top_inter_num = len(inter_indices[0])
                iou = top_inter_num / (len(pred_indices) + len(gt_indices) - top_inter_num)
        pred_ious.append(iou)
    return np.array(pred_ious)


if __name__ == '__main__':
    vis = False

    aps = []
    ars = []
    for i in range(10):
        scene_names = os.listdir(EVAL_DIR)
        confusions = []
        precisions = []
        recalls = []
        for name in scene_names:
            scene_dir = os.path.join(EVAL_DIR, name)
            jsondata_path = os.path.join(scene_dir, name + ".json")
            obj_path = os.path.join(scene_dir, name + ".obj")
            if not os.path.exists(obj_path):
                obj_path = os.path.join(scene_dir, name + ".pcd") #selecting pcd if obj not present
            mesh = o3d.io.read_triangle_mesh(obj_path)
            to_display = [mesh]

            # GROUND TRUTH
            with open(jsondata_path, 'r') as f:
                jsondata = json.load(f)

            bboxes, bbox_labels = json2bboxes(jsondata)
            to_display.extend(bboxes)

            assert 'point_lines' in jsondata
            point_lines = json2trajectories(jsondata)

            mesh_lines = create_mesh_lines(point_lines)
            to_display.extend(mesh_lines)

            # PREDICTION
            predicted_welding_paths, predicted_bboxes, point_cloud = welding_paths_detection(obj_path, vis=vis)
            to_display.extend(predicted_bboxes)

            if len(predicted_welding_paths) > 0:

                predicted_point_lines = predicted_welding_paths[0][:, :, :3, 3]
                predicted_mesh_lines = create_mesh_lines(predicted_point_lines)
                for i in predicted_mesh_lines:  i.paint_uniform_color([0,1,0])
                to_display.extend(predicted_mesh_lines)

            if vis: o3d.visualization.draw_geometries(to_display)

            bboxes_iou = calculate_bboxes_iou(point_cloud, predicted_bboxes, bboxes)

            iou_thresh = 0.8
            TP = len(bboxes_iou[bboxes_iou >= iou_thresh])
            FP = len(bboxes_iou) - TP
            FN = len(bboxes) - TP

            precision = TP/(TP+FP)
            recall = TP/(TP+FN)

            confusions.append([TP, FP, FN])
            precisions.append(precision)
            recalls.append(recall)

            # print(point_lines)
            # if len(predicted_welding_paths) > 0:
            #     print(predicted_point_lines)

            print('----------------SCENE "', name, '" EVALUATION----------------')
            print("Bounding Boxes IoUs:   ", bboxes_iou)
            print("\n")
            print("        POSITIVES   NEGATIVES")
            print("TRUE       ", TP)
            print("FALSE      ", FP, "        ", FN)
            print("\n")
            print("PRECISION: ", precision)
            print("RECALL:    ", recall)
            print("\n")


        AP = np.sum(precisions)/len(precisions)
        AR = np.sum(recalls)/len(recalls)
        aps.append(AP)
        ars.append(AR)

        print('------------------------------------------')
        print("AVERAGE PRECISION: ", AP)
        print("AVERAGE RECALL:    ", AR)

    MAP = np.sum(aps) / len(aps)
    MAR = np.sum(ars) / len(ars)
    print('--*************************************************--')
    print("MEAN AVERAGE PRECISION: ", MAP)
    print("MEAN AVERAGE RECALL:    ", MAR)
