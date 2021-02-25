## Autonomous Navigation in Complex Environments with Deep Multimodal Fusion Network - IROS 2020


### System and softwares information

- Ubuntu 16.04/18.04

- ROS Kinetic/Medolic

- Gazebo 7/8


### How to load the world files

- Download the [3D models](https://drive.google.com/file/d/1xva5lNi6kNoubPUL0KQ0YvST-ZIC4ml2/view?usp=sharing) (3GB), then extract and copy them to your Gazebo folder located at `/home/<use_name>/.gazebo/models`.

- Copy world files inside `world_files` folder into your catkin folder.

- The example world file is in `navigation/simulation-env/world_files` folder.

- Example command to run: `rosrun gazebo_ros gazebo world_files/house4.world`.


### How to generate domain randomisation with random texture

- Make sure you pull all models from this git and copy them into your Gazebo model folder (which is located in `/.gazebo/models`)

- Make sure you had backed up your orginal models in `./gazebo/models`. This is important since the code will delete your original textures and replace them. You will lose your original textures if you don't back up them!

- Open GENERATE.py and edit the path into your path of Gazebo models folder.

- Run the GENERATE.py file: `python GENERATE.py`  This will automatically replace all old textures by new textures

- Reload your world files. You should see your worlds are in new colors.

- Known issues: Gazebo may immediately crash when loading a model or a world file. Fix this with: `sudo apt-get install libgazebo7-dev`


### Build BeetleBot
    $ cd navigation/beetlebot

    $ catkin_make

    $ . devel/setup.bash

    $ roslaunch beetlebot_gazebo beetlebot_world.launch

    (Wait until the world finishes loading in the gazebo)

    $ roslaunch beetlebot_gazebo load_robot.launch

### Control robot with keyboard:

    $ cd navigation/beetlebot
    $ . devel/setup.bash
    $ cd src/beetlebot_AIOZ/beetlebot_control/scripts
    $ python keyboard_controller.py

key| command|
---|---|
w|move forward
a| turn left|
d| turn right|
s| move backward|
space|stop
r| rotate delivery-robot clockwise
f| rotate delivery-robot counter-clockwise
i| increase velocity by 1
k| decrease velocity by 1
j| decrease turning radius by 10 or increase turning degree
l| increase turning radius by 10 or decrease turning degree

### Dataset
- Download the navigation dataset from [here](https://drive.google.com/file/d/168V_f4a1wo6H-fHTkVQRvxZkthaYfEdH/view?usp=sharing) (58GB)

### Training
To be update


### Citation

If you use this code as part of any published research, we'd really appreciate it if you could cite the following paper:

```
@article{IROS2020aioz,
  author    = {Anh Nguyen and
               Ngoc Nguyen and
               Kim Tran and
               Erman Tjiputra and
               Quang D. Tran},
  title     = {Autonomous Navigation in Complex Environments with Deep Multimodal
               Fusion Network},
  journal   = {CoRR},
  volume    = {abs/2007.15945},
  year      = {2020}
}

@article{ROSBook2021aioz,
author = {Nguyen, Anh and Tran, Quang},
year = {2020},
month = {12},
journal = {ROSBook 2021 - Springer Book on Robot Operating System (ROS)},
title = {Autonomous Navigation with Mobile Robots using Deep Learning and the Robot Operating System}
}
```

### License
AIOZ License

### More information
AIOZ AI Homepage: https://ai.aioz.io
AIOZ BeetleBot: https://beetle.aioz.io
