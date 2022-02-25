
# Arena-Rosnav (IROS 21)

**Note:**
For our 3D version using Gazebo as simulation platform, please visit our [arena-rosnav-3D repo](https://github.com/ignc-research/arena-rosnav-3D)

A flexible, high-performance 2D simulator with configurable agents, multiple sensors, and benchmark scenarios for testing robotic navigation.

A flexible, high-performance 2D simulator with configurable agents, multiple sensors, and benchmark scenarios for testing robotic navigation.

Arena-Rosnav uses Flatland as the core simulator and is a modular high-level library for end-to-end experiments in embodied AI -- defining embodied AI tasks (e.g. navigation, obstacle avoidance, behavior cloning), training agents (via imitation or reinforcement learning, or no learning at all using conventional approaches like DWA, TEB or MPC), and benchmarking their performance on the defined tasks using standard metrics.

| <img width="400" height="400" src="/img/rosnav1.gif"> | <img width="400" height="400" src="/img/rosnav2.gif"> |
| :---------------------------------------------------: | :---------------------------------------------------: |
|                   _Training Stage_                    |                  _Deployment Stage_                   |

## What is this repository for?

Train DRL agents on ROS compatible simulations for autonomous navigation in highly dynamic environments. Flatland-DRL integration is inspired by Ronja Gueldenring's work: [drl_local_planner_ros_stable_baselines](https://github.com/RGring/drl_local_planner_ros_stable_baselines.git). Test state of the art local and global planners in ROS environments both in simulation and on real hardware. Following features are included:

- Setup to train a local planner with reinforcement learning approaches from [stable baselines3](https://github.com/DLR-RM/stable-baselines3.git)
- Training in simulator [Flatland](https://github.com/avidbots/flatland) in train mode
- Include realistic behavior patterns and semantic states of obstacles (speaking, running, etc.)
- Include different obstacles classes (other robots, vehicles, types of persons, etc.)
- Implementation of intermediate planner classes to combine local DRL planner with global map-based planning of ROS Navigation stack
- Testing a variety of planners (learning based and model based) within specific scenarios in test mode
- Modular structure for extension of new functionalities and approaches

# Start Guide

We recommend starting with the [start guide](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/guide.md) which contains all information you need to know to start off with this project including installation on **Linux and Windows** as well as tutorials to start with.

- For Mac, please refer to our [Docker](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Docker.md).

## 1. Installation

Open the terminal with `Ctrl`+`Alt`+`T` and enter below commands one at a time.

In order to check the details of the easy installation script, please refer to the [script file](https://raw.githubusercontent.com/ignc-research/arena-rosnav/noetic-devel/setup.sh).

```bash
sudo apt-get update && sudo apt-get upgrade
wget https://raw.githubusercontent.com/ignc-research/arena-rosnav/noetic-devel/setup.sh -O - | bash
```

Create a virtual environment

```bash
source ~/.bashrc && mkvirtualenv --python=python3.8 rosnav
```

Install further dependencies (you can take a look at the script [here](https://raw.githubusercontent.com/ignc-research/arena-rosnav/noetic-devel/setup2.sh))

```bash
wget https://raw.githubusercontent.com/ignc-research/arena-rosnav/noetic-devel/setup2.sh -O - | bash
source ~/.bashrc && workon rosnav
```

Now everything should be set up. You can start the simulation with:

```bash
roslaunch arena_bringup start_arena_flatland.launch
```

Alternatively, refer to [Installation.md](docs/Installation.md) for detailed explanations about the installation process.

## 1.1. Docker

We provide a Docker file to run our code on other operating systems. Please refer to [Docker.md](docs/Docker.md) for more information.

## 2. Usage

### DRL Training

Please refer to [DRL-Training.md](docs/DRL-Training.md) for detailed explanations about agent, policy and training setups.

### Scenario Creation with the [arena-scenario-gui](https://github.com/ignc-research/arena-scenario-gui/)

To create complex, collaborative scenarios for training and/or evaluation purposes, please refer to the repo [arena-scenario-gui](https://github.com/ignc-research/arena-scenario-gui/). This application provides you with an user interface to easily create complex scenarios with multiple dynamic and static obstacles by drawing and other simple UI elements like dragging and dropping. This will save you a lot of time in creating complex scenarios for you individual use cases.

# IROS21 information

To test the code and reproduce the experiments, follow the installation steps in [Installation.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Installation.md). Afterwards, follow the steps in [Evaluations.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Evaluations.md).

To test the different **Waypoint Generators**, follow the steps in [waypoint_eval.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/eval_28032021.md)

**DRL agents** are located in the [agents folder](https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/agents).

# Used third party repos:

- Flatland: http://flatland-simulator.readthedocs.io
- ROS navigation stack: http://wiki.ros.org/navigation
- Pedsim: https://github.com/srl-freiburg/pedsim_ros

# Arena Sound 
## Overview
Arena-Rosnav has been extended to include an audio simulation in its 2D environment. In this integration of a sound simulation
the dynamic obstacles of each scenario, that is to say the Pedsim agents, are going to produce a specific set of sounds according to their social state at each time of the simulation. In this context the navigating agent/robot will have the role of perceiving those sound signals in its environment and reproduce them through the system's default audio interface. In order for this sound reproduction to be done realistically, the position and the movement of all agents should affect the way the perceived sound is rendered.

## OpenAL
Considering those requirements the use of the OpenAL 3D audio API (http://openal.org/) was found to be a very suitable option. This library models a collection of audio sources moving in a 3D space that are heard by a single listener somewhere in that space. The basic OpenAL objects are a Listener, a Source, and a Buffer. There can be a large number of Buffers, which contain audio data. Each buffer can be attached to one or more Sources, which represent points in 3D space which are emitting audio. There is always one Listener object, which represents the position where the Sources are heard -- rendering is done from the perspective of the Listener.

Given those capabilities enabled by the OpenAL Library, we can integrate its functionality into our simulation by using the Listener object for our navigation agent/robot and the Sources for the Pedsim agents accordingly. Furthermore we are using a number of Buffers, each one of them being loaded with audio data corresponding to one of the social states a Pedsim agent can take.

<img width="400" height="400" src="/img/openal_diagram.png"> 

The implemantation used for the OpenAL Library is OpenAL-Soft (https://openal-soft.org/). The latest release version is 1.21.1

# Demo

#### 1. Add Arena Sound to a launch file

Add the following to the launch file you want to use:
```
  <arg name="enable_sound" default="true"/>
  
  <node name="sound_manager" pkg="arena_sound_manager" type="sound_manager_node" output="screen" if="$(arg enable_sound)">
  </node>
```
#### 2. Add flatland's SoundPlugin in the model connected to the Listener

Add the following to the robot yaml model you are using (e.g. simulator_setup/robot/myrobot.model.yaml) in order to connect it with the OpenAL's Listener object
```
plugins:
  - type: SoundPlugin
    name: sound_plugin
    body: base_footprint
```

The body parameter corresponds to the main body's name of the model.
#### 3. Add flatland's PedsimSound Plugin in the models connected to the Sources

Add the following to the Pedsim agents yaml models (e.g. simulator_setup/dynamic_obstacles/person_two_legged_sound.model.yaml) included in the Scenario you are using in order to connect them to OpenAL's Sources objects.
```
plugins:
  - type: PedsimSound
    name: pedsim_sound
    body: base
    gain: 1.0
```

* The body parameter corresponds to the main body's name of the model.
* The gain parameter is a scalar amplitude multiplier for the connected Source object's emiting sounds. 
    * the default 1.0 means that the sound is unattenuated 
    * a value of 0.5 is equivalent to an attenuation of 6 dB
    * zero equals silence
    * gain larger than 1.0 causes an amplification for the Source
