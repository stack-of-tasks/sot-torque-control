# Overview {#index}
<!--
/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */
-->

\section OverviewIntro Introduction to sot-torque-control

[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Building Status](https://travis-ci.org/stack-of-tasks/sot-torque-control.svg?branch=master)](https://travis-ci.org/stack-of-tasks/sot-torque-control)
[![Pipeline status](https://gepgitlab.laas.fr/stack-of-tasks/sot-torque-control/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/stack-of-tasks/sot-torque-control/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/stack-of-tasks/sot-torque-control/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/sot-torque-control/master/coverage/)

The library sot-torque-control contains a collection of dynamic-graph entities aimed at implementing torque control on different robots.
You can find a description of the main entities in the following <a href="https://github.com/stack-of-tasks/sot-torque-control/wiki">wiki</a>.

## Dependencies
This project depends on:
* [dynamic-graph](https://github.com/jrl-umi3218/dynamic-graph) >= 3.0.0
* [dynamic-graph-python](https://github.com/stack-of-tasks/dynamic-graph-python) >= 3.0.0
* [sot-core](https://github.com/stack-of-tasks/sot-core) >= 3.0.0
* [pinocchio](https://github.com/stack-of-tasks/pinocchio) >= 1.2
* [tsid](https://github.com/stack-of-tasks/tsid) >= 1.2
* [PinInvDyn](https://github.com/stack-of-tasks/invdyn)
* [parametric-curves](https://github.com/stack-of-tasks/parametric-curves)
* [simple_humanoid_description](https://github.com/laas/simple_humanoid_description)(for unit testing)

All of these packages (except PinInvDyn) can be installed through [robotpkg](http://robotpkg.openrobots.org/).
In particular, you can find them in [robotpkg-wip](http://robotpkg.openrobots.org/robotpkg-wip.html) (work in progress), a subset of robotpkg.

You will need the sot-talos and talos-dev packages:
```
sudo apt-get install robotpkg-py27-sot-talos robotpkg-talos-dev
```

Pay attention not to install ROS using robotpkg though, because it would install the latest version, which may not be what you need.


You can find the full installation procedure in the <a href="md_doc_installation.html">installation page</a>.

Instructions for running a simulation of Pyrene executing a CoM sinusoid in position or torque control can be found <a href="md_doc_running.html">here</a>.

Instructions for running a simulation or an experiment using the DDP on the right elbow of Pyrene can be found <a href="md_doc_ddpRun.html">here</a>.

Instructions for running a simulation of Pyrene executing a foot sinusoid in the air in torque control can be found <a href="md_doc_bellStepRun.html">here</a>.

Instructions for running a simulation of Pyrene walking in torque control can be found <a href="md_doc_walkRun.html">here</a>.
