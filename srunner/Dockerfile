FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu16.04

RUN git clone https://github.com/felipecode/coiltraine.git  && cd coiltraine && \
              conda env create -f requirements.yaml

#export PYTHONPATH=<Path-To-CARLA-Latest>/PythonAPI/carla-0.9.3-py3.5-linux-x86_64.egg:/<Path-To-CARLA-Latest>/PythonAPI:$PYTHONPATH


CMD python3 view_model.py  -f baselines -e resnet34imnet -cp 180000 -cv 0.9