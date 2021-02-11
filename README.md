# RoboWeldAR Weld Seam Trajectory Detection


![Alt text](weld_seam_proposal.png?raw=true "Weld seam trajectory proposal")

## Description

This module is part of the [RoboWeldAR]() project, and can be used to obtain a set of welding trajectories (in the form of line segments with pose) from a 3D model of rectangular metal panels.

## Pipeline Description:

The pipeline consists of the following submodules:

- [Facebook's Votenet](https://github.com/facebookresearch/votenet)[[2]](#2) Deep Neural Network for 3-D object detection. A modified implementation of the original is used, that is modified to train on a new dataset containing panels. Details of the modifications, the new dataset, and the source code are included and described [here](https://github.com/ikh-innovation/roboweldar-votenet).

- A client module that uses websocket and HTTP found [here](./networking/client.py). It is used to interface with the main [RoboWeldAR coordinator module](https://github.com/ikh-innovation/roboweldar-networking), which coordinates the entire RoboWeldAR workflow. The client module, when executed, expects the ip of the server as an argument (`--host`), which will connect to. Then, it waits for a message from the server to download the mesh files produced by the previous step of RoboWeldAR (3D reconstruction). When the mesh is downloaded, the pipeline proccess is triggered, which produces weld seam trajectory proposals for the referent mesh. These trajectories are stored in a numpy file and consequtively are uploaded to the server. This whole proccess, including the pipeline, can be dockerized. The instructions and further information can be found [here](https://github.com/ikh-innovation/roboweldar-dih-deliverables/tree/main/T2/roboweldar.weld_seam_detection/docker). 

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

![Alt text](pipeline.png?raw=true "Weld seam detection pipeline")

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

- Model evaluation runs inference on all of the test data. The evaluation dataset is available in #TODO. 


#Installation
In an environment containing an installation of Python 3.6 and Pip, follow these steps:
- pip -r install requirements.txt
- pip -r install votenet/requirements.txt
- cd votenet/pointnet2/ && python setup.py install



## References
<a id="1">[1]</a> 
Bazazian, Dena, Josep R. Casas, and Javier Ruiz-Hidalgo. "Fast and robust edge extraction in unorganized point clouds." 2015 international conference on digital image computing: techniques and applications (DICTA). IEEE, 2015.

<a id="2">[2]</a>
Qi, Charles R., et al. "Deep hough voting for 3d object detection in point clouds." Proceedings of the IEEE/CVF International Conference on Computer Vision. 2019.
