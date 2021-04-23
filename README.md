# RoboWeldAR Weld Seam Trajectory Detection


![Alt text](weld_seam_proposal.png?raw=true "Weld seam trajectory proposal")

## Description

This repository contains the weld seam trajectory proposal module, which is part of the RoboWeldAR project. It can be used as standalone, or as part of the  [RoboWeldAR ROSE-AP](https://github.com/ikh-innovation/roboweldar-rose-ap) application. Given a 3D model of a welding scene, this module detects the welding targets (metallic panels) and produces a feasable set of welding trajectories (in the form of line segments with pose).

## Quickstart
There are three possible ways to use this module:
- Using the pre-made docker image is available at [dockerhub](https://hub.docker.com/repository/docker/roboweldar/roboweldar-weld-seam-detection).
- Building the docker image. Follow the instructions at [docker](docker/README.md).
- Building and running the application locally. PLease read the [architecture description](docs/architecture.md), follow the the installation instructions below.

Docker images are made such as to communicate with the [coordinator module](https://github.com/ikh-innovation/roboweldar-networking) via an API, which is described [here](docs/api.md). If the source code is used locally, the same behavior can be achieved by running [client](networking/client.py). If the purpose is to read and export from a local file or to change the parameters and tweak the algorithms, start by running the [pipeline](seam-detection/pipeline.py) file, and refer to [api](docks/api.md).


## Installation
for building and running the module manually, a setup with a CUDA GPU available is required, with installed CUDNN developers kit, and the relevant drivers. In an environment containing an installation of Python 3.6, pip and git, follow these steps:
- git clone --recurse-submodules https://github.com/ikh-innovation/roboweldar-weld-seam-detection.git
- pip -r install requirements.txt
- pip -r install votenet/requirements.txt
- cd votenet/pointnet2/ && python setup.py install



