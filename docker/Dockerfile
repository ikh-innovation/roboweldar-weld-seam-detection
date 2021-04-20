FROM nvidia/cuda:10.2-cudnn8-devel-ubuntu18.04
RUN apt-get update

#REQUIREMENTS
RUN apt-get install -y wget python3-pip python3 libgl1-mesa-glx git
RUN apt-get install -y gcc-6 g++-6 g++-6-multilib gfortran-6
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6
RUN pip3 install --upgrade pip
RUN mkdir -p /opt

#ROBOWELDAR REPO DOWNLOAD
ADD repo-key /
RUN \
  chmod 600 /repo-key && \
  echo "IdentityFile /repo-key" >> /etc/ssh/ssh_config && \
  echo "StrictHostKeyChecking no" >> /etc/ssh/ssh_config

WORKDIR /opt/
ENV GITHUB_SSH_REPO=git@github.com:ikh-innovation/roboweldar-weld-seam-detection.git
RUN git clone --recurse-submodules --single-branch ${GITHUB_SSH_REPO}
#--single-branch --branch docker_branch

#CUDA
RUN export LD_LIBRARY_PATH=/usr/local/cuda/lib64

#SETUP
RUN yes | pip3 install -r roboweldar-weld-seam-detection/requirements.txt
RUN yes | pip3 install -r roboweldar-weld-seam-detection/votenet/requirements.txt

WORKDIR /opt/roboweldar-weld-seam-detection

RUN cd votenet/pointnet2 && python3 setup.py install

ADD sample_reconstruction /opt/roboweldar-weld-seam-detection/sample_reconstruction
ADD checkpoint /opt/roboweldar-weld-seam-detection/votenet/log_panelnet/selected_checkpoint

#PORTS
EXPOSE 3000
EXPOSE 3001

#EXECUTE
ENV PYTHONPATH "${PYTHONPATH}:$(pwd)"

ENTRYPOINT ["python3", "networking/client.py"]