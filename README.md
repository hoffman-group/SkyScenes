<div align="center">
  
# SkyScenes: A Synthetic Dataset for Aerial Scene Understanding
[Sahil Khose](https://sahilkhose.github.io/)\*, [Anisha Pal](https://anipal.github.io/)\*, [Aayushi Agarwal](https://www.linkedin.com/in/aayushiag/)\*, [Deepanshi](https://www.linkedin.com/in/deepanshi-d/)\*, [Judy Hoffman](https://faculty.cc.gatech.edu/~judy/), [Prithvijit Chattopadhyay](https://prithv1.xyz/)
</div>

<!-- This repository is the official Pytorch implementation for [SkyScenes](). -->

[![HuggingFace Dataset](https://img.shields.io/badge/🤗-HuggingFace%20Dataset-cyan.svg)](https://huggingface.co/datasets/hoffman-lab/SkyScenes) [![Project Page](https://img.shields.io/badge/Project-Website-orange)](https://github.gatech.edu/pages/hoffman-group/SkyScenes/) [![arXiv](https://img.shields.io/badge/arXiv-SkyScenes-b31b1b.svg)]()  


<!-- [![Watch the Demo](./assets/robust_aerial_videos.mp4)](./assets/robust_aerial_videos.mp4) -->

<img src="./assets/skyscene_intro_teaser.png" width="100%"/>


## 🚀 Release Update

- Code under maintanence, will be released soon!


## 💻 Installation and Generation

- Step 0: Install docker: https://docs.docker.com/engine/install/ubuntu/

    Check out [CARLA's documentation](https://carla.readthedocs.io/en/latest/build_docker_unreal/) on how to setup docker for further details. 

- Step 1: Setup docker
    ```bash
    sudo systemctl start docker
    sudo docker pull carlasim/carla:0.9.14
    sudo docker run --privileged --gpus all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlasim/carla:0.9.14 /bin/bash ./CarlaUE4.sh -RenderOffScreen
    ```

- Step 2: Open a new terminal 

    To get `[containerName]`:
    ```bash
    sudo docker ps # under NAMES
    ```
    ```bash
    sudo docker cp [containerName]:/home/carla/PythonAPI ./
    ```

- Step 3: Data Generation

    Update the DIR to store the data along with the various height, pitch, town, weather variations inside this script.
    ```bash
    python3 generate_variations.py
    ```


## 🔧 Troubleshooting

- If you have already started a docker container and you want to gracefully stop it to re-run the commands for generation:
    ```bash
    sudo docker ps
    sudo docker stop [NAME or ID]
    ```
- *Note: Every script should have the following snippet at the beginning before importing carla*
    ```
    import sys
    sys.path.append('PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg')
    import carla
    ```

    *We already have these paths in our generation scripts.*
    *You might have to change linux to windows/mac according to your system*
- Generating images
    
    We have already generated the inital datapoints using this and saved the metaData under `./meta_data`. The `humanspawn()` algorithm is located in this script.
    ```bash
    python3 manualSpawning.py
    ```

