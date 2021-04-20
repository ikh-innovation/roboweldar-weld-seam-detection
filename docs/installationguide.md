## Install

This section contains installation instructions for the RoboWeldAR tracking component.

## Requirements

Prior to running the entrypoint `track_devices.py`, you must make sure that the correct dependencies are installed.

# Hardware

Make sure that the [HTC Vive Pro Full Kit](https://www.vive.com/uk/product/vive-pro-full-kit/) is powered up and properly connected to the host workstation, and that your workstation meets [these minimum requirements](https://www.vive.com/us/support/vive-pro-hmd/category_howto/what-are-the-system-requirements.html).

We recommend a Linux Ubuntu 18.02 workstation (Linux version 5.3.0-62-generic kernel), with a GeForce GTX 1050 graphics card, as our testing was carried out there. 


# Software

* Install the latest [NVIDIA CUDA](https://developer.nvidia.com/cuda-downloads) drivers.
* Install the [Steam runtime](https://store.steampowered.com/about/).
* From within Steam, install SteamVR.
* Set up the peripherals from within the Steam UI, including the room setup.
* To install Python 3.6, run:
```text
sudo add-apt-repository ppa:deadsnakes && \
sudo apt-get update && \
sudo apt-get install python3.6 python3-pip
```
* Install the dependencies
```text
python3.6 -m pip install venv
cd roboweldar.tracking
python3.6 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```
* Run the script and capture some data!
```text
cd roboweldar.tracking
source .venv/bin/activate
export PYTHONPATH=$PYTHONPATH:$(pwd)
python src/track_devices.py
```
Note: the entrypoint's default arguments and more detailed usage parameters are given in [User & Programmers Manual](usermanual.md).



