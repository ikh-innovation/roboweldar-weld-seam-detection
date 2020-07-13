# flask-server.py - a minimal flask api using flask_restful
from flask import Flask, request
from flask_restful import Resource, Api
import open3d as o3d
import json
import numpy as np
from prediction import edge_detection
app = Flask(__name__)
api = Api(app)
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

class SeamDetector(Resource):
    def get(self):
        return "Post a list of voxels to get the edge prediction"

    def post(self):
        pcd_json = request.get_json()

        points = np.array(pcd_json['points'])

        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(points)
        pointcloud.paint_uniform_color([0.5,0.5,0.5])

        predicted_pointcloud, edges_pointcloud = edge_detection(pointcloud)

        o3d.io.write_point_cloud('/mount/predicted_pointcloud.ply', predicted_pointcloud)

        return "pointcloud predicted."

api.add_resource(SeamDetector, '/pointcloud')

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
