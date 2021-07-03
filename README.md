# Routing Optimization Platform for Autonomous Tractors
## Introduction

## Getting Started

### Pre requisites
Ubuntu 18.04.5 LTS (Bionic Beaver)

ROS Melodic (Morenia)

#### Setting up ROS Melodic "Morenia"

**1. Install the dependencies step to step.**

* $ sudo apt install python3-pip
* $ sudo apt-get install python-rosdep

**1.1 Install ROS Melodic following all the steps taken [here]** http://wiki.ros.org/melodic/Installation/Ubuntu

Install a ROS velodyne simulator package:
* $ sudo apt-get install ros-melodic-velodyne-simulator

**1.2 Create a ROS Workspace.**

* $ mkdir -p ~/catkin_ws/src
* $ cd ~/catkin_ws/
* $ catkin_make
* $ source devel/setup.bash

**note:** if catkin command (catkin_make) not work try:
* $ sudo apt-get install ros-melodic-catkin

#### Setting up ROS tractor_sim simulation

**2. Install the dependencies for simulation step to step.**

**2.1 state_controller**
* $ cd ~/catkin_ws/src
* $ git clone https://github.com/olinrobotics/state_controller.git
* $ cd ..
* $ rosdep install -iry --from-paths src
* $ cd ~/catkin_ws/
* $ catkin_make
* $ source devel/setup.bash

**2.2 GRAVL - install with the following all the steps taken [here]** https://github.com/olinrobotics/gravl

**2.3 tractor_sim_packages**
* $ cd ~/catkin_ws/src
* $ git clone https://github.com/olinrobotics/tractor_sim_packages.git
* $ cd ..
* $ rosdep install -iry --from-paths src
* $ cd ~/catkin_ws/
* $ catkin_make
* $ source devel/setup.bash

**2.4 tractor_sim**
* $ cd ~/catkin_ws/src
* $ git clone https://github.com/olinrobotics/tractor_sim.git
* $ cd ..
* $ rosdep install -iry --from-paths src
* $ cd ~/catkin_ws/
* $ catkin_make
* $ source devel/setup.bash

**Note:** To use the models included in this repo, copy the contents of the folder to ~/.gazebo/models

* $ cd ~/catkin_ws/src/tractor_sim/tractor_sim_gazebo/models
* $ cp -R . ~/.gazebo/models

### Installation
@Eduardo Nascimento: Como clonar, catking build, colocar world e meshes disponíveis etc...
## Usage
@Eduardo Nascimento
## Roadmap
Planos para o futuro, melhorias, etc.
proposta de fazer funcionar o nav_goal como melhoria?, a tradução de um nó em outro? porque ninguém fez isso ainda?.
## Acknowledgments
Agradecer aos professores e todos que participaram e ajudaram direta ou indiretamente.
