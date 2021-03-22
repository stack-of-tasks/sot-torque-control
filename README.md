# sot-torque-control

[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Building Status](https://travis-ci.org/stack-of-tasks/sot-torque-control.svg?branch=master)](https://travis-ci.org/stack-of-tasks/sot-torque-control)
[![Pipeline status](https://gitlab.laas.fr/stack-of-tasks/sot-torque-control/badges/master/pipeline.svg)](https://gitlab.laas.fr/stack-of-tasks/sot-torque-control/commits/master)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/sot-torque-control/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/sot-torque-control/master/coverage/)

Collection of dynamic-graph entities aimed at implementing torque control on different robots.
Read the project wiki to get a description of the main entities contained in this repository.

## Dependencies
This project depends on:
* dynamic-graph >= 3.0.0
* dynamic-graph-python >= 3.0.0
* sot-core >= 3.0.0
* pinocchio >= 1.2
* [parametric-curves](https://github.com/stack-of-tasks/parametric-curves)
* (for unit testing)[example-robot-data](https://github.com/gepetto/example-robot-data) >= 3.8.0

All of these packages (except PinInvDyn) can be installed through [robotpkg](http://robotpkg.openrobots.org/).
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
