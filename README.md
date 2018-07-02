# **modelica_bridge**
## Introduction
***modelica_bridge*** is the [ROS](www.ros.org) package complement of the [Modelica](www.modelica.org) package [***ROS_Bridge***](https://github.com/ModROS/ROS_Bridge.git).

The purpose of ***modelica_bridge*** is to provide an interface between Modelica tools and ROS. This interface, or bridge, is accomplished by running tcp/ip sockets in a ROS node, and in an external C-function through Modelica (accomplished via ***ROS_Bridge***).

## Usage Summary

The modelica_bridge package consists of the node connecting ROS and Modelica, **modbridge_node**, and the message type **modbridge_node** publishes on, the **ModComm.msg**. 

The package can take up to 256 inputs from the ROS controllers, and can send a maximum of 1024 characters over a character buffer to Modelica. It has two parameters: *port_num* and *update_rate*. Ensure that the *port_num* matches the value of the Modelica model's port number parameter.

For more detailed documentation, see the ROS [wiki](wiki.ros.org/modelica_bridge).

## **modelica_bridge** Development Summary
### 0.1.1 (2018-07-02)
- Fixes issues with documentation
- Fixes issue of *CMakeLists.txt* not having install targets
- Cleans up and resolves *package.xml* dependencies list
#
### 0.1.0 (2018-06-30)
- Indexed and released to ros-buildfarm using **bloom**
- Added [wiki for **modelica_bridge**](wiki.ros.org/modelica_bridge)
- Added Doxygen documentation using rosdoc_lite
#
### 0.0.0 (2018-06-30)
- First full commit of the **modelica_bridge** package.
  - Previously known as **modros**