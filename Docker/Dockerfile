FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu16.04
FROM python:3.7

RUN apt-get update && apt-get install -y --no-install-recommends \
         build-essential \
         cmake \
         git \
         curl \
         vim \
         ca-certificates \
         libjpeg-dev \
         libpng-dev &&\
     rm -rf /var/lib/apt/lists/*


RUN curl -o ~/miniconda.sh -O  https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh  && \
     chmod +x ~/miniconda.sh && \
     ~/miniconda.sh -b -p /opt/conda && \
     rm ~/miniconda.sh && \
     /opt/conda/bin/conda install -y python=3.5 numpy pyyaml scipy ipython mkl mkl-include cython typing && \
     /opt/conda/bin/conda install -y -c pytorch magma-cuda100 && \
     /opt/conda/bin/conda clean -ya


RUN git clone https://github.com/felipecode/coiltraine.git && \
    cd coiltraine \
    RUN /opt/conda/bin/conda env create -f requirements.yaml \
    RUN echo "source activate $(head -1 requirements.yaml | cut -d' ' -f2)" > ~/.bashrc


ENV PATH /opt/conda/envs/$(head -1 requirements.yaml | cut -d' ' -f2)/bin:$PATH

# Set the directory as the new work dir
#WORKDIR /usr/coiltraine
#RUN pwd

#ADD requirements.yaml /tmp/requirements.yaml
#RUN /opt/conda/bin/conda env create -f /tmp/requirements.yaml
# Pull the requirements name out of the requirements.yml
#RUN echo "source activate $(head -1 /tmp/requirements.yaml | cut -d' ' -f2)" > ~/.bashrc
#ENV PATH /opt/conda/envs/$(head -1 /tmp/requirements.yaml | cut -d' ' -f2)/bin:$PATH


RUN git clone https://github.com/carla-simulator/carla.git


RUN export PYTHONPATH=carla-0.9.4-py3.5-linux-x86_64.egg:/carla/PythonAPI:$PYTHONPATH


CMD python3 /tmp/coiltraine/view_model.py  -f baselines -e resnet34imnet -cp 180000 -cv 0.9

