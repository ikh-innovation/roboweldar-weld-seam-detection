# RoboWeldAR Seam Detection

## Description

This module is part of the [RoboWeldAR]() project, and can be used to obtain a set of welding trajectories (in the form of line segments with pose) from a 3D model of rectangular metal panels.

## Pipeline Description:

The pipeline consists of the following submodules:

- Computer vision 3-D edge detection algorithm implementation of [[1]](#1). The source code for this is located [here](./seam-detection/algorithms.py).

- Facebook's Votenet algorithm for 3-D object detection. A modified implementation of the original, including a panel dataset generator, is located [here](https://github.com/ikh-innovation/roboweldar-votenet).

- A client module that uses websocket and HTTP found [here](./networking/client.py) , which is used to interface with the main [RoboWeldAR coordinator module](https://github.com/ikh-innovation/roboweldar-networking), which coordinates the entire RoboWeldAR workflow.

- The main pipeline is located [here](./seam-detection/pipeline.py). The entry point `welding_paths_detection()` runs the following:
    - Loads the input, which is a mesh file (.obj), or a point cloud (.pcd).
    - The pre-processing step discards the irrelevant points located outside the scanning hemisphere of the robot's working envelope (using `reconstruction_filter()`). (The value of the length is hard-coded and probably should be a parameter passed into the pipeline from the scannning step.) Furthermore, if the file is a mesh, it is sampled into a point cloud.
    - The detection step, where objects(panels) within the input scenes are detected and consecutively, weld seams are proposed. It can be split in the following sections: 
        - Detection of vertical and horizontal panels via pointlcoud inference of the customized VoteNet model (`rbw_inference()`).
        - From the the VoteNet's result, [Open3D](http://www.open3d.org) compatible bounding boxes are created (`parse_bounding_boxes()`). The bounding boxes that contain less points than a threshold (`min_points_2b_empty`=700) are also filtered.
        - The `panel_registration()` function is used to better align the predicted bounding boxes from previous step to the closest set of points (representing a panel) in the point cloud. This uses Point-to-Plane Iterative Closest Point (ICP) algorithm.
        - The `edge_detection()` function is applied to the pointcloud, to detect which points are on edges. It is an implementation of [[1]](#1) found [here](./seam-detection/algorithms.py).
        - The `edges_within_bboxes()` function is used, to find out which edge points are contained within each of the detected bounding boxes.
        - The `points_intersection()` function is then applied, to find the intersections of the prementioned edge points between each combination of panels.
        - Finally, the function `detect_trajectories()` computes linear segments from the intersecting points between panels, using RANSAC algorithm. It accompanies the results with an optimal pose with which the robot's end-effector should approach each linear segment. This is serialized into numpy (.npy) format for use by the next steps of the RoboWeldAR workflow. The array is the following format: Nx2x4x4. N is the number of segments, 2 for each pair of start and end point of each segment, and a 4x4 matrix representing the transformation matrix for each point position, including its pose. 

## Pipeline Evaluation

### Evaluation dataset
To evaluate the performance of object detection and trajectory proposal, a small curated set of unseen welding scenes was constructed that includes:
- 15 Hand-picked generated scenes that are close to real examples, and explore different situations of configurations.
- 3 real-world reconstructed scenes, that were scanned by the robotic arm, and another 7, that were manually augmented and transformed from these three, in order to simulate errors and the noise that a reconstruction might produce.
- 2 scenes with more complicated panel configurations, that were made as well as if they had been reconstructed in the robot's simulation environment.

### Manual Scene Annotation
- To annotate the bounding boxes in the non-generated 3D reconstructions, [Supervisely](http://www.supervise.ly) was used. This tool produced a json file that contained the location, size and class of each of the annotations.

- To annotate the ground-truth weld seams in all of the scenes, a custom tool was developed, which can be found [here](./seam-detection/trajectory_annotator.py). Using Open3D's interface, the user can select each time two points, to represent a line segment. The proccess is repeated N times, indicated by `lines_num` parameter. The annotations are added to the previously mentioned json files if they exist in case of reconstructed scenes, and in case of generated scenes, they are created in similar fashion. 

### Object Detection and Weld Seam Estimation Evaluation
Apart from the evaluation intergraded in original Votenet's code, a custom [evaluation](./seam-detection/evaluation.py) file was built to run the complete pipeline for each of the scenes in eval dataset and to measure and print out the average Precision and Recall for both panel detection and weld seam estimation. This evaluation is run 10 times to find a consistent average, due to the non-deterministic nature of the algorithms. 

- Model evaluation runs inference on all of the test data (`welding_scenes_eval` dir). #TODO: need to think how to include large file storage (Git LFS / OneDrive extension?)

## Data Generation and Training






## TODO

- See above TODOs.
- DONE Move our version of VoteNet into IKH repository on GitHub. 
- DONE Make the necessary changes to the imports in the code.
- DONE Make argument parsers for the following:
    1. Main entry point (pipeline.py).
    2. Evaluation code
    3. Trajectory annotator

- Explain modifications made to VoteNet (in separate README.md?)


        




#Installation
In an environment containing an installation of Python 3.6 and Pip, follow these steps:
- pip -r install requirements.txt
- pip -r install votenet/requirements.txt
- cd votenet/pointnet2/ && python setup.py install

#Execution?





## References
<a id="1">[1]</a> 
Bazazian, Dena, Josep R. Casas, and Javier Ruiz-Hidalgo. "Fast and robust edge extraction in unorganized point clouds." 2015 international conference on digital image computing: techniques and applications (DICTA). IEEE, 2015.
