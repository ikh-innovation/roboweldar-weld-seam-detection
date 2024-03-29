import os
import shutil
import threading
import glob
import time
from functools import partial
import simplejson as json
import sys
import numpy as np
import open3d as o3d

# TODO: import the following from roboweldar-networking
from roboweldar_networking.interfaces import ws_client
from roboweldar_networking.interfaces import http_client
from roboweldar_networking.interfaces.http_client import send_files



BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'seam-detection'))
OUTPUT_DIR = os.path.join(BASE_DIR, "welding_paths_output")
OUTPUT_NPY = os.path.join(OUTPUT_DIR, "welding_paths.npy")
OUTPUT_SIMPLIFIED_MESH = os.path.join(OUTPUT_DIR, "simplified_mesh.obj")
OUTPUT_RAJECTORY_MESH = os.path.join(OUTPUT_DIR, "trajectories_mesh.obj")
MESH_DIR = os.path.join(BASE_DIR, "reconstructed_mesh")
MESH_FILE = os.path.join(MESH_DIR, "transformed_mesh.obj")

from pipeline import welding_paths_detection
from simplify_mesh import simplify_mesh
# from welding_path_generation import *

is_done = False

def get_mesh_files(host, httpPort, path_to_dir, mesh_files=["transformed_mesh.obj", "transformed_mesh.mtl", "transformed_mesh_0.png"]):
    # Default mesh files: ["transformed_mesh.obj", "transformed_mesh.mtl", "transformed_mesh_0.png"]
    files = http_client.get_filenames('http://' + str(host) + ':' + str(httpPort) + '/' + 'mesh_filenames')
    print(files)
    try:
        [files.index(file) for file in mesh_files]
    except ValueError as err:
        print("Not all files are uploaded yet!")
        return False

    for _file in mesh_files:
        url = 'http://' + str(host) + ':' + str(httpPort) + '/serve_mesh_files?name=' + str(_file)
        content = http_client.download_file(url)
        path_to_file = os.path.join(path_to_dir, str(_file))
        with open(path_to_file, 'wb') as f:
            print("Writing mesh: {}".format(path_to_file))
            f.write(content)

    return True

def clean_up_folder(path_to_dir: str):
    files = glob.glob(os.path.join(path_to_dir, "*"))
    for f in files:
        if os.path.isdir(f):
            shutil.rmtree(f)
        else:
            os.remove(f)


def create_folder(path_to_dir: str):
    if os.path.isdir(path_to_dir) or os.path.isfile(path_to_dir):
        pass
    else:
        os.mkdir(path_to_dir)

def unify_meshes(mesh_list):
    ''' unifies a list of meshes together, into one mesh. Texture is not implemented'''

    unified_mesh = o3d.geometry.TriangleMesh()
    triangles =[]
    normals = []
    vertices = []
    vertex_normals = []
    vertex_colors = []
    for mesh in mesh_list:
        triangles.extend(np.asarray(mesh.triangles) + len(vertices))
        normals.extend(np.asarray(mesh.triangle_normals))
        vertices.extend(np.asarray(mesh.vertices))
        vertex_normals.extend(np.asarray(mesh.vertex_normals))
        vertex_colors.extend(np.asarray(mesh.vertex_colors))

    unified_mesh.triangles = o3d.utility.Vector3iVector(triangles)
    unified_mesh.triangle_normals = o3d.utility.Vector3dVector(normals)
    unified_mesh.vertices = o3d.utility.Vector3dVector(vertices)
    unified_mesh.vertex_normals = o3d.utility.Vector3dVector(vertex_normals)
    unified_mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)

    return unified_mesh


def start():
    #TODO execute pipeline
    wpaths, trajectories = welding_paths_detection(MESH_FILE, vis=False, vis_out=True)
    np.save(OUTPUT_NPY, wpaths)
    mesh = o3d.io.read_triangle_mesh(MESH_FILE)
    simplified_mesh = simplify_mesh(mesh)
    o3d.io.write_triangle_mesh(OUTPUT_SIMPLIFIED_MESH, simplified_mesh)

    # trajectories.append(mesh)
    one_mesh_trajectories = unify_meshes(trajectories)
    o3d.io.write_triangle_mesh(OUTPUT_RAJECTORY_MESH, one_mesh_trajectories)

    #TODO TEMPORARY RUN WITHOUT PIPELINE
    # load = np.load("/home/innovation/Projects/roboweldar-weld-seam-detection/seam-detection/welding_trajectories.npy")
    # np.save(OUTPUT_NPY, load)

    global is_done
    is_done = True
    message = "Done welding path detection..."
    print(message)

    return message


def stop():
        # TODO: Clean all files after module is stopped
        return "Stopped StructureFromMotion module..."

def  on_message(ws, message: str, host: str, port: str):
    d = json.loads(message)
    if d["message"] == "start_weld_seam_detection":
        # get the mesh from the server
        print("Downloading mesh from the server ({}) to {}...".format(host, MESH_DIR))
        get_mesh_files(host=host, httpPort=port, path_to_dir=MESH_DIR)
        print("Starting Welding path detection...")
        start()
    elif d["message"] == "stop":
        print("Stopping Welding path detection...")
        stop()


def main(host, endpoint):
    global is_done
    # make sure dirs exist
    create_folder(MESH_DIR)
    create_folder(OUTPUT_DIR)

    # clean up directory
    clean_up_folder(MESH_DIR)
    clean_up_folder(OUTPUT_DIR)

    # get_mesh_files(host=host, httpPort=3000, path_to_dir=MESH_DIR)
    # start()

    # init client
    wsClient = ws_client.getClient("ws://" + host + ":3001/" + endpoint)
    print("ws://" + host + ":3001/" + endpoint)
    wsClient.on_message = partial(on_message, host=host, port=3000)
    wst = threading.Thread(target=wsClient.run_forever)
    wst.daemon = True
    wst.start()

    while(True):
        # print(is_done)
        time.sleep(1)
        if not is_done: continue
        url = "http://" + str(host) + ":3000/cache_welding_trajectory"
        print("Uploading welding paths numpy array to {}...".format(url))
        is_sent_mesh = send_files(url, [OUTPUT_NPY, OUTPUT_SIMPLIFIED_MESH, OUTPUT_RAJECTORY_MESH])
        print("Uploaded welding paths numpy array to {}...".format(url))
        is_done = False

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default="192.168.51.61", help="Host on which the server is running")
    args = parser.parse_args()

    main(host=args.host, endpoint="weld_seam_detection")
