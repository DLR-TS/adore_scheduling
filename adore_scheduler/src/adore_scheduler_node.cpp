/********************************************************************************
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
 ********************************************************************************/

#include <adore_scheduler.h>

#include <iostream>
#include <thread>

bool paused;
bool terminated;
adore_if_ros_scheduling::Scheduler<unsigned int, std::pair<uint32_t, uint32_t>> *scheduler_;
void kbinput()
{
    while (!terminated)
    {
        char c = std::cin.get();
        switch (c)
        {
        case 's':
        {
            scheduler_->start();
        }
        break;
        case 'p':
        {
            scheduler_->togglePause();
        }
        break;
        case 'i':
        {
            scheduler_->printInfo();
        }
        break;
        case 'l':
        {
            scheduler_->limitSimulationSpeed();
        }
        break;
        }
    }
}

void checkParameters(ros::NodeHandle n, bool &autostart)
{
    bool parameter_set;
    bool parameter_value = false;

    parameter_set = n.getParam("/use_sim_time", parameter_value);
    if (parameter_set)
        std::cout << "Parameter /use_sim_time is set to " << (parameter_value ? "true." : "false.") << std::endl;
    else
        std::cout << "Parameter /use_sim_time is not set." << std::endl;

    parameter_set = n.getParam("/tcp_no_delay", parameter_value);
    if (parameter_set)
        std::cout << "Parameter /tcp_no_delay is set to " << (parameter_value ? "true." : "false.") << std::endl;
    else
        std::cout << "Parameter /tcp_no_delay is not set." << std::endl;

    parameter_set = n.getParam("/use_scheduler", parameter_value);
    if (parameter_set)
        std::cout << "Parameter /use_scheduler is set to " << (parameter_value ? "true." : "false.") << std::endl;
    else
        std::cout << "Parameter /use_scheduler is not set." << std::endl;

    parameter_set = n.getParam("/autostart_scheduling", autostart);
    if (parameter_set)
        std::cout << "Parameter /autostart_scheduling is set to " << (autostart ? "true." : "false.") << std::endl;
    else
        std::cout << "Parameter /autostart_scheduling is not set." << std::endl;

    parameter_set = n.getParam("/render", parameter_value);
    if (parameter_set)
        std::cout << "Parameter /render is set to " << (parameter_value ? "true." : "false.") << std::endl;
    else
        std::cout << "Parameter /render is not set." << std::endl;
}

int main(int argc, char **argv)
{
    terminated = false;
    std::thread kbinput_thread(kbinput);
    ros::init(argc, argv, "adore_scheduler_node");
    ros::NodeHandle n;
    bool autostart = false;
    checkParameters(n, autostart);
    int numberOfNodes = 0;
    n.getParam("/number_of_nodes", numberOfNodes);
    std::cout << "number of nodes " << numberOfNodes << std::endl;
    scheduler_ = adore_if_ros_scheduling::Scheduler<unsigned int, std::pair<uint32_t, uint32_t>>::getInstance(
        n, numberOfNodes, autostart, true);
    scheduler_->init();
    while (n.ok())
    {
        ros::spin();
    }
    terminated = true;
    kbinput_thread.join();
    return 0;
}
