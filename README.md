<!--
********************************************************************************
* Copyright (C) 2017-2023 German Aerospace Center (DLR). 
* Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
*
* This program and the accompanying materials are made available under the 
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0.
*
* SPDX-License-Identifier: EPL-2.0 
*
* Contributors: 
*   Matthias Nichting - initial API and implementation
********************************************************************************
-->

# adore_scheduling
This repository provides scheduling functionalities for [ADORe](https://github.com/eclipse/adore) and its ROS interface. It contains the subprojects lib_adore_scheduling, adore_if_ros_scheduling_msg, adore_if_ros_scheduling and adore_scheduler.

## Build Status
[![CI](https://github.com/DLR-TS/adore_scheduling/actions/workflows/ci.yaml/badge.svg)](https://github.com/DLR-TS/adore_scheduling/actions/workflows/ci.yaml)

## Getting Started
You must have make and docker installed.

1. git clone with submodules:
```bash
git clone git clone --recursive -j8 git@github.com:DLR-TS/adore_scheduling.git
```
2. build
```bash
make build
```
