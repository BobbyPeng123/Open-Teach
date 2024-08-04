#!/bin/bash
conda activate crossmodal
export PATH="/home/raunaq/miniconda3/envs/crossmodal/bin:$PATH"
mkdir -p logs
nohup /home/raunaq/miniconda3/envs/crossmodal/bin/python robot_camera.py > logs/camera_log.txt &
cd server
nohup /home/raunaq/miniconda3/envs/crossmodal/bin/gunicorn -w 12 -b 0.0.0.0:5000 -k gevent --timeout 0 --worker-connections 2 'monitor:app' > ../logs/cam_server.txt &