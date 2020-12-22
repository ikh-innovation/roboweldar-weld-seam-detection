import open3d as o3d
import json
import numpy as np
import os
from lineMesh import LineMesh


def json2bboxes(data: json) -> [o3d.geometry.OrientedBoundingBox, []]:
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


def json2trajectories(data: json):
    return json.loads(data['point_lines'])


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


def pick_points(pcd):
    print("")
    print("1) Please pick two correspondences using [shift + left click]")
    print("     Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window and then choose the next points")
    vis = o3d.visualization.VisualizerWithVertexSelection()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def create_mesh_lines(point_lines):
    mesh_lines = []
    for line in point_lines:
        trajectories = LineMesh([line[0], line[1]], [[0, 1]], [[1, 0, 0]], radius=0.002)
        trajectories_segments = trajectories.cylinder_segments
        for seg in trajectories_segments:
            seg.compute_vertex_normals()
            mesh_lines.extend(trajectories_segments)
    return mesh_lines


def choose_trajectories(pcd, lines_num=2):
    '''open visualiser to enable user to pick 2 points that represent a line'''
    point_lines = []
    for i in range(lines_num):
        picked = pick_points(pcd)
        if len(picked) != 2:
            print("wrong number of points picked. Exiting...")
            quit()
        point_lines.append([picked[0].coord.tolist(), picked[1].coord.tolist()])

    return point_lines


if __name__ == '__main__':
    # data_path = "/home/innovation/Downloads/labeled_welding_scenes"
    data_path = "/home/innovation/Projects/roboweldar-weld-seam-detection/seam-detection/welding_scenes_eval"
    scene_names = sorted(list(os.listdir(data_path)))

    for name in scene_names:
        print("\n", name, " is selected.\n")
        selected_scene_name = name #scene_names[4]
        selected_scene = os.path.join(data_path, selected_scene_name, selected_scene_name)

        with open(selected_scene + ".json", 'r') as f:
            jsondata = json.load(f)

        bboxes, bbox_labels = json2bboxes(jsondata)
        to_display = bboxes

        if 'point_lines' in jsondata:
            # print("Trajectories detected. Visualizing...")
            # mesh_lines = create_mesh_lines(json2trajectories(jsondata))
            # to_display.extend(mesh_lines)
            # o3d.visualization.draw_geometries(to_display)
            # quit()
            print("Trajectories detected. Skipping...")
            continue

        if os.path.exists(selected_scene + ".obj"):
            obj = o3d.io.read_triangle_mesh(selected_scene + ".obj")
            obj.compute_vertex_normals()
            to_display.append(obj)
            pcd = obj.sample_points_uniformly(5000000)
        else:
            pcd = o3d.io.read_point_cloud(selected_scene + ".pcd")

        to_display.append(pcd)

        point_lines = choose_trajectories(pcd, 2)

        mesh_lines = create_mesh_lines(point_lines)
        to_display.extend(mesh_lines)

        o3d.visualization.draw_geometries(to_display)

        # save to json file
        jsondata['point_lines'] = json.dumps(point_lines)
        with open(selected_scene + ".json", 'w') as f:
            json.dump(jsondata, f, indent=4)
