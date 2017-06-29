# sot-torque-control
Collection of dynamic-graph entities aimed at implementing torque control on different robots.
Read the project wiki to get a description of the main entities contained in this repository.

## Dependencies
This project depends on:
* jrl-mal >= 1.8.0
* dynamic-graph >= 1.0.0
* dynamic-graph-python
* sot-core
* pinocchio >= 1.2
* metapod
* [tsid](https://github.com/stack-of-tasks/tsid)

All of these packages can be installed through [robotpkg](http://robotpkg.openrobots.org/).
In particular, you can find them in [robotpkg-wip](http://robotpkg.openrobots.org/robotpkg-wip.html) (work in progress), a subset of robotpkg.
Pay attention not to install ROS using robotpkg though, because it would install the latest version, which may not be what you need
(e.g. currently on HRP-2 we are using ROS indigo).

## Installation

    git clone --recursive git@github.com:stack-of-tasks/sot-torque-control.git
    cd sot-torque-control
    mkdir _build-RELEASE
    cd _build-RELEASE
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=$DEVEL_DIR/openrobots
    make install
    
In Ubuntu 14.04 you may need to add `-DCMAKE_CXX_FLAGS="-std=c++11"`.
