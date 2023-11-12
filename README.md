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
This repository provides scheduling functionalities for [ADORe](https://github.com/eclipse/adore) and its ROS interface. It contains the two ROS2 packages adore_basenode and adore_scheduler. Together they allow a stepped simulation of ADORe.

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
## adore_basenode
adore_basenode is a library that provides the class `adore::scheduling::BaseNode`. It is recommended to use the class as a parent class for ROS2 nodes. This enables communication with adore_scheduler needed for stepped simulations. However, it does not hinder the possiblity of classical non-stepped simulation. An example of the use of this class is shown in the following source code:

```c++
        void BaseNode::addTimerCallback(std::shared_ptr<std::function<void()>> callbackFcn, unsigned int frequency_divisor)
        {
            m_callbacks.push_back(std::make_pair(std::make_pair(frequency_divisor, frequency_divisor - 1), callbackFcn));

        }
```


Here, ExampleNode has two visible callbacks, callback_1 and callback_2. A third callback, which is called directly by the BaseNode, is not visible. The scheduler is notified within this callback so that the latter can increase the time signal. callback_1 is added using the BaseNode::addTimerCallback function. This type of callback definition ensures that the scheduler is only notified once all callbacks added in this way have been completed. When adding directly via XYZ, as in the example with callback_2, this is not guaranteed, so that the scheduler can be notified before or after callback_2 is called. Callbacks added via BaseNode::addTimerCallback are always executed in the order in which they were added via BaseNode::addTimerCallback. The method has an optional argument, BaseNode::addTimerCallback(callback, frquency_divisor = 1). If frequency_divisor is set, the corresponding callback is called with accordingly reduced frequency.
