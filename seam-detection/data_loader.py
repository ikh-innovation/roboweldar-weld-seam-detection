import open3d as o3d
import json
import numpy as np
import os


def json2bboxes(data: json):
    '''Author:lefteris '''
    labelboxes = []
    object_list = []
    for i in range(len(data["figures"])):
        obb = np.zeros((8))
        obb[0] = data["figures"][i]["geometry"]["position"]["x"]
        obb[1] = data["figures"][i]["geometry"]["position"]["y"]
        obb[2] = data["figures"][i]["geometry"]["position"]["z"]
        obb[3] = data["figures"][i]["geometry"]["dimensions"]["x"] / 2
        obb[4] = data["figures"][i]["geometry"]["dimensions"]["y"] / 2
        obb[5] = data["figures"][i]["geometry"]["dimensions"]["z"] / 2
        obb[6] = data["figures"][i]["geometry"]["rotation"]["z"]

        if obb[2] > 0.01:
            obb[7] = 0
        else:
            obb[7] = 1

        boxx = box_from_points(obb)
        labelboxes.append(boxx)
        object_list.append(obb)
    return labelboxes, object_list


def box_from_points(obb):
    '''Author:lefteris '''
    corners = np.zeros((8, 3))
    if obb[7] == 1:
        corners[0] = [obb[0] - obb[3], obb[1] - obb[4], obb[2] - obb[5]]
        corners[1] = [obb[0] - obb[3], obb[1] + obb[4], obb[2] - obb[5]]
        corners[2] = [obb[0] + obb[3], obb[1] - obb[4], obb[2] - obb[5]]
        corners[3] = [obb[0] - obb[3], obb[1] - obb[4], obb[2] + obb[5]]
        corners[4] = [obb[0] + obb[3], obb[1] + obb[4], obb[2] + obb[5]]
        corners[5] = [obb[0] + obb[3], obb[1] - obb[4], obb[2] + obb[5]]
        corners[6] = [obb[0] - obb[3], obb[1] + obb[4], obb[2] + obb[5]]
        corners[7] = [obb[0] + obb[3], obb[1] + obb[4], obb[2] - obb[5]]
    else:
        corners[0] = [obb[0] - obb[3], obb[1] - obb[4], obb[2] - obb[5]]
        corners[1] = [obb[0] + obb[3], obb[1] - obb[4], obb[2] - obb[5]]
        corners[2] = [obb[0] - obb[3], obb[1] + obb[4], obb[2] - obb[5]]
        corners[3] = [obb[0] - obb[3], obb[1] - obb[4], obb[2] + obb[5]]
        corners[4] = [obb[0] + obb[3], obb[1] + obb[4], obb[2] + obb[5]]
        corners[5] = [obb[0] - obb[3], obb[1] + obb[4], obb[2] + obb[5]]
        corners[6] = [obb[0] + obb[3], obb[1] - obb[4], obb[2] + obb[5]]
        corners[7] = [obb[0] + obb[3], obb[1] + obb[4], obb[2] - obb[5]]

    box3d = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(corners))
    box3d.rotate(o3d.geometry.get_rotation_matrix_from_xyz((0, 0, obb[6])), np.array([obb[0], obb[1], obb[2]]))

    return box3d



if __name__ == '__main__':
    data_path = "/home/thanos/Downloads/labeled_welding_scenes"
    scene_names = sorted(list(os.listdir(data_path)))

    selected_scene_name = scene_names[50]
    selected_scene = os.path.join(data_path, selected_scene_name, selected_scene_name)

    with open(selected_scene + ".json") as f:
        jsondata = json.load(f)

    print(jsondata)
    bboxes, bbox_labels = json2bboxes(jsondata)
    pcd = o3d.io.read_point_cloud(selected_scene + ".pcd")

    bboxes.append(pcd)
    o3d.visualization.draw_geometries(bboxes)



