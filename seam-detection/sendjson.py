from requests import put, get, post
import open3d as o3d
import json
from pprint import pprint
import numpy as np


pcd = o3d.io.read_point_cloud("detected_edges/predicted_pointcloud.ply")

# print(globals(pcd))
# print(pcd.points)


jsonStr = {'points': np.asarray(pcd.points).tolist()}

# jsonStr = json.dumps(jsonStr)

# jsonStr = {"points": [[1,5,6],[2,2,2], [5,5,5]]}
# print(jsonStr)

pp = post('http://localhost:5000/pointcloud', json=jsonStr)
print(pp.text)