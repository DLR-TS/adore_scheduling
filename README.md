<!--
********************************al objects std::cout and std::wcout control output to a stream buffer of implementation-defined type (derived from std::streambuf), associated with the standard C output stream stdout. ************************************************
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
This repository provides scheduling functionalities for [ADORe](https://github.com/eclipse/adore). It contains the two ROS2 packages adore_basenode and adore_scheduler. Together they allow a stepped simulation of ADORe.

## Build Status
[![CI](https://github.com/DLR-TS/adore_scheduling/actions/workflows/ci.yaml/badge.svg)](https://github.com/DLR-TS/adore_scheduling/actions/workflows/ci.yaml)
<!--
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
-->

## adore_basenode
adore_basenode is a library that provides the class `adore::scheduling::BaseNode`. It is recommended to use the class as a parent class for ROS2 nodes. This enables the communication with adore_scheduler needed for stepped simulations. However, it does not hinder the possiblity of classical non-stepped simulation. The constructor takes the time between two consequtive starts of the periodically called run loop  of type `std::chrono::duration` (any typedef) and the node name of type `std::string`. An example of the use of this class is shown in the following source code:


```c++
#include <adore_basenode/basenode.h>

class ExampleNode : public adore::scheduling::BaseNode
{
public:
    ExampleNode(std::chrono::milliseconds duration_between_fcn_calls, std::string nodename)
        : adore::scheduling::BaseNode(duration_between_fcn_calls, nodename)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("example_topic", 10);
        auto run_fcn_1 = std::make_shared<std::function<void()>>(std::bind(&ExampleNode::timer_callback_1, this));
        BaseNode::addTimerCallback(run_fcn_1);
        last_time_1 = 0;
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    double last_time_1;
    void timer_callback_1()
    {
        auto message = std_msgs::msg::String();
        double now = this->now().seconds();
        message.data = "freqDivisor = 1 " + std::to_string(now - last_time_1);
        last_time_1 = now;
        publisher_->publish(message);
        std::cout << message.data << std::endl;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExampleNode>(std::chrono::milliseconds(100), "examplenode"));
    rclcpp::shutdown();
    return 0;
}
```

Here, `ExampleNode` has one visible callback. A second callback which is called directly by the `BaseNode` is not visible. This second callback is to inform the scheduler that the execution is finished for the current point in time. `callback_1` is added using the `BaseNode::addTimerCallback` function. This type of callback definition ensures that the scheduler is only notified once all callbacks added in this way have been completed. When adding callbacks directly as, e.g., via `rclcpp::create_timer(...)`, the order of execution is not guaranteed, the callback to notify the scheduler could be called before the callback that performs any actual tasks. Callbacks added via `BaseNode::addTimerCallback` are always executed in the order in which they were added via this function. The function has an optional argument, `BaseNode::addTimerCallback(callback, frquency_divisor = 1)`. If frequency_divisor is set, the corresponding callback is called with accordingly reduced frequency. In order to use the scheduling functionality, the `use_scheduler` parameter bust be set to `True` on startup. 

## adore_scheduler
The `adore_scheduler` is a ROS2 node that does the actual scheduling for all nodes that have registered. The scheduling can be paused/resumed by typing `'p'`, the simulation speed can be limited to ~1.0 by typing `'l'`. Typing `'i'` makes the scheduler writing an informational list of registered nodes to stdout. The list contains the upper bound in time for every registered node which might be especially helpful for debugging in certain circumstances.
