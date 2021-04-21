FROM nvidia/cuda:10.2-cudnn8-devel-ubuntu18.04
RUN apt-get update

#REQUIREMENTS
RUN apt-get install -y wget python3-pip python3 libgl1-mesa-glx git
RUN apt-get install -y gcc-6 g++-6 g++-6-multilib gfortran-6
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6
RUN pip3 install --upgrade pip

#RUN mkdir -p /opt/roboweldar-weld-seam-detection
WORKDIR /opt/roboweldar-weld-seam-detection

COPY seam-detection .
COPY votenet .
COPY networking .
COPY requirements.txt .

#CUDA
RUN export LD_LIBRARY_PATH=/usr/local/cuda/lib64

#SETUP
RUN yes | pip3 install -r requirements.txt
RUN yes | pip3 install -r votenet/requirements.txt

RUN cd votenet/pointnet2 && python3 setup.py install

ADD sample_reconstruction sample_reconstruction
ADD checkpoint votenet/log_panelnet/selected_checkpoint

#PORTS
EXPOSE 3000
EXPOSE 3001

#EXECUTE
ENV PYTHONPATH "${PYTHONPATH}:$(pwd)"

ENTRYPOINT ["python3", "networking/client.py"]