# Docker Image ReadMe

The [Dockerfile](Dockerfile) associated with this image can be used to build an image of the weld seam detection module that is waiting to receive a 3D mesh model of a welding scene from a server and a message to begin the the detection. When it is finished, it returns an array of welding paths.

## Requirements

#### Usage
To use this image, a coordinator server to orchestrate the procedure is required. A server as such is provided in [roboweldar.coordinator](https://github.com/ikh-innovation/roboweldar-dih-deliverables/blob/main/T2/roboweldar.coordinator). 

#### Deployment
To deploy the image, apart from Docker, the following are required:

1. A computer that has an NVIDIA GPU with CUDA cores.

1. [CUDA Drivers](https://developer.nvidia.com/cuda-downloads). Tested with driver 440 - Cuda 10.2 and driver 450 - Cuda 11.

1. [NVIDIA docker](https://github.com/NVIDIA/nvidia-docker). It can be downloaded by following the instructions from the official website or by running the [quick installation script](docker_cuda_install_script.sh) for Ubuntu.

## How to build an image

The `Dockerfile` uses an [NVIDIA Docker image](https://hub.docker.com/r/nvidia/cuda/) as base, [Alicevision's Merhroom](https://alicevision.org/#meshroom) runtime version 2019.2.0. While inside the root directory of the module, to build the image run

```console
docker build -f docker/Dockerfile -t <component-name> .

```
## How to run the image

Running the image requires to expose ports `3000` and `3001` to the network and the argument `--host` to show the IP of the server. 
The following are two sample cases:

1. Running the image standalone:
```console
docker run --runtime=nvidia -p 3000:3000 -p 3001:3001  -d <component-name> --host <server-ip>
```

1. Running the image in the same host as the server and using the local network:
```console
docker run --runtime=nvidia --network host -d <component-name> --host "localhost"
```
