# RoboWeldAR Seam Detection

## Description

This module is aprt of the [RoboWeldAR]() project, and can be used to obtain a set of welding trajectories (in the form of line segments) from a 3D model of rectangular metal panels.

## Pipeline

The pipeline consists of the following submodules:

- Computer vision 3-D edge detection algorithm implementation of [[1]](#1). The source code for this is located [here](./seam-detection/algorithms.py).

- Facebook's Votenet algorithm for 3-D object detection. The source code for this will be located [here](./votenet). #TODO

- A client module that uses websocket and HTTP found [here](./networking/client.py) , which is used to interface with the main [RoboWeldAR coordinator module](https://github.com/ikh-innovation/roboweldar-networking), which coordinated the entire RoboWeldAR workflow.

- The main pipeline is located [here](./seam-detection/pipeline.py). The entry point `welding_paths_detection()` runs the following:
    - Loads the input, which is a mesh file (.obj), or point cloud (.pcd).
    - The pre-processing step discards the irrelevant points located outside the scanning hemisphere of the robot's working envelope (using `reconstruction_filter()`). This is hard-coded and probably should be a parameter passed into the pipeline from the scannning step. #TODO
    - The detection step finds the edges using the algorithm in  [[1]](#1). 
        - It detects the panels in the pointcloud via the inference of the VoteNet model `rbw_inference()`. This is located in the VoteNet source dir but should probably be included in this repository. #TODO
        - The VoteNet result is processed, such that a [Open3D](http://www.open3d.org) compatible bounding box is created (`parse_bounding_boxes()`).
        - Then, the `panel_registration()` function is used to better align the predicted panels from previous step to the closest set of points (representign a panel) in the point cloud. This uses Point-to-Plane Iterative Closest Point (ICP) algorithm.
        - Then, the `edges_within_bboxes()` functions is used to ..............................
        - The function `points_intersection()` finds all the common points between the bounding boxes. 
        - ...........................................................
        - The function `detect_trajectories()` computes linear segments from the intersecting points between panels, using RANSAC algorithm. It accompanies the results with an optimal pose with which the robot's end-effector should approach each linear segment. This is serialized into numpy (.npy) format for use by the next steps of the RoboWeldAR workflow.


## Trajectory Annotator and Model Evaluation

### Trajectory Annotation

- Used [Supervisely](http://www.supervise.ly) to annotate some real world scans, all based on one model of a welding panel, as well as a simulated welding panel. Basic steps to reproduce the annotation process:
    1. #TODO
    2. #TODO
    3. #TODO

- .................

### Model Evaluation



## Data Generation and Training






## TODO

- See above TODOs.
- Move our version of VoteNet into IKH repository on GitHub. 
- Make the necessary changes to the imports in the code.
- Make argument parsers for the following:
    1. Main entry point (pipeline.py).
    2. Evaluation code
    3. Trajectory annotator
- Fix `requirements.txt`


        










## References
<a id="1">[1]</a> 
Bazazian, Dena, Josep R. Casas, and Javier Ruiz-Hidalgo. "Fast and robust edge extraction in unorganized point clouds." 2015 international conference on digital image computing: techniques and applications (DICTA). IEEE, 2015.
