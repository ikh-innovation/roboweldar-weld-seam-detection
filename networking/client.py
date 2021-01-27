import os
import shutil
import threading
import glob
import time
from functools import partial
import simplejson as json
import sys
import numpy as np

# TODO: import the following from roboweldar-networking
from roboweldar_networking.interfaces import ws_client
from roboweldar_networking.interfaces import http_client
from roboweldar_networking.interfaces.http_client import send_files



BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'seam-detection'))
OUTPUT_DIR = os.path.join(BASE_DIR, "welding_paths_output")
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "welding_paths.npy")
MESH_DIR = os.path.join(BASE_DIR, "reconstructed_mesh")

print(BASE_DIR)
# from pipeline import welding_paths_detection
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


def start():

    #TODO execute pipeline
    # wpaths, _ = welding_paths_detection("/home/innovation/Projects/meshroom_workspace/reconstruction_2/transformed_mesh/transformed_mesh.obj")
    # np.save(wpaths, OUTPUT_FILE)

    #TODO TEMPORARY RUN WITHOUT PIPELINE
    load = np.load("/home/innovation/Projects/roboweldar-weld-seam-detection/seam-detection/welding_trajectories.npy")
    np.save(OUTPUT_FILE, load)

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
        print(is_done)
        time.sleep(1)
        if not is_done: continue
        url = "http://" + str(host) + ":3000/cache_welding_trajectory"
        print("Uploading welding paths numpy array to {}...".format(url))
        is_sent_mesh = send_files(url, [OUTPUT_FILE])
        print("Uploaded welding paths numpy array to {}...".format(url))
        is_done = False
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', required=True,
                        help="Host on which the server is running")

    args = parser.parse_args()
    main(host=args.host, endpoint="weld_seam_detection")
