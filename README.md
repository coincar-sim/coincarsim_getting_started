[![Build Status](https://api.travis-ci.org/coincar-sim/coincarsim_getting_started.svg?branch=release)](https://travis-ci.org/coincar-sim/coincarsim_getting_started)

# CoInCar-Sim
### An Open-Source Simulation Framework for Cooperatively Interacting Automobiles

This is the main readme of the ROS Simulation Framework CoInCar-Sim, developed jointly within the [Priority Program 1835 "Cooperatively Interacting Automobiles"](http://www.coincar.de/) of the German Science Foundation (DFG) and within the [TechCenter A-Drive](http://tcadrive.de/) funded by the state of Baden-Württemberg.

![Visualization of the framework (rviz)](doc/framework-rviz.png)

The motivation and concept is explained in our [publication](http://www.mrt.kit.edu/z/publ/download/2018/Naumann2018CoInCarSim.pdf) [[DOI](http://dx.doi.org/%2010.1109/IVS.2018.8500405)].

## General Information
Installation requires Linux as operating system. The framework is developed and tested under [Ubuntu 16.04](http://releases.ubuntu.com/16.04/).

#### Prerequisites
In order to use the framework, you need to install the following packages (installable via `apt install`)
* ROS (see http://wiki.ros.org/ROS/Installation)
  * `ros-kinetic-desktop-full`
  * `ros-kinetic-tf2-geometry-msgs`
* Catkin tools and wstool
  * `python-catkin-tools` (http://catkin-tools.readthedocs.io/en/latest/index.html)
  * `python-wstool` (http://wiki.ros.org/wstool)
* System Libraries
  * `libgeographic-dev` (for lanelet2)
  * `libpugixml-dev` (for lanelet2)
  * `qt5-default`
  * boost, boost-python and eigen are installed with ROS

## Installation
This section describes the installation of the framework, assuming the prerequisites have already been installed.

For the full installation starting at a clean Ubuntu 16.04, have a look at how Travis does it in about 15 minutes: [![Build Status](https://api.travis-ci.org/coincar-sim/coincarsim_getting_started.svg?branch=release)](https://travis-ci.org/coincar-sim/coincarsim_getting_started)

#### 1) Source ROS
* open a terminal
* source ROS
  * `source /opt/ros/kinetic/setup.bash`
  * you might want to add this to your `.bashrc`

#### 2) Use the script to download and install the framework
* navigate to the folder where you want to place the framework
* execute the script
  * `desired_folder$ bash <(wget -qO- https://raw.githubusercontent.com/coincar-sim/coincarsim_getting_started/release/setup_workspace.sh)`
  * this creates a catkin workspace and downloads all [source dependencies](OVERVIEW_COMPONENTS.md) as defined in [`simulation_framework_latest.rosinstall`](ws_config/simulation_framework_latest.rosinstall)

### Alternative Installation

#### 2a) Download the script and execute the local one
* download the desired `*.rosinstall` file from the folder `ws_config` (or clone this repo)
  * this config file contains the url of all [source dependencies](OVERVIEW_COMPONENTS.md) for their automated download in step 3
  * recommended: use `simulation_framework_latest.rosinstall`
* also download the script and do the above steps by starting
  * ` $ ./setup_workspace.sh simulation_framework_latest.rosinstall`  

#### 2b) Set up and build the workspace yourself
* download the desired `*.rosinstall` file from the folder `ws_config` (or clone this repo)
  * this config file contains the url of all [source dependencies](OVERVIEW_COMPONENTS.md) for their automated download in step 3
  * recommended: use `simulation_framework_latest.rosinstall`
* set up a catkin workspace from the `*.rosinstall` file you just downloaded
  * ` $ mkdir catkin_ws && cd catkin_ws`
  * `catkin_ws$ wstool init src PATH_TO_ROSINSTALL_FILE.rosinstall`
* build the workspace
  * `catkin_ws$ catkin build`
* source the build-files
  * `catkin_ws$ source devel/setup.bash`

## Usage
#### 3) Launch the framework
* source the build-files
  * `catkin_ws$ source devel/setup.bash`
* start the simulation framework by launching the main launchfile:
  * `catkin_ws$ roslaunch simulation_initialization_ros_tool _whole_framework.launch`
  * see the Readme of simulation_initialization_ros_tool for details about how the parts of the framework are launched

#### 4) Play with it
* as further described in [FIRST_STEPS.md](FIRST_STEPS.md)

#### 5) Contribute
* fork simulation_initialization_ros_tool and modify the launchfiles/settings/workspace config to simulate (parts of) your vehicle
* fork other components or create new vehicles to modify and/or extend the functionality

#### 6) Stay tuned
* if you did choose the latest version, you can update it with
  * `catkin_ws$ wstool update`

## License
See the respective packages for license issues.

## Citation
If you are using CoInCar-Sim for scientific research, we would be pleased if you would cite our [publication](http://www.mrt.kit.edu/z/publ/download/2018/Naumann2018CoInCarSim.pdf) [[DOI](http://dx.doi.org/%2010.1109/IVS.2018.8500405)]:
```latex
@inproceedings{Naumann2018CoInCarSim,
  title     = {{C}o{I}n{C}ar-{S}im: An Open-Source Simulation Framework for Cooperatively Interacting Automobiles},
  author    = {M. {Naumann} and F. {Poggenhans} and M. {Lauer} and C. {Stiller}},
  booktitle = {Proc. IEEE Int. Conf. Intelligent Vehicles},
  year      = {2018},
  address   = {Changshu, China},
  month     = {June},
  pages     = {1879--1884},
  doi       = {10.1109/IVS.2018.8500405}
}
```
