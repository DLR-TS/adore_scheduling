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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#include "schedulernotificationmanager.h"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace adore
{
namespace scheduling
{
    /**
     *  Base class for ros nodes - BaseNode provides functions that can be used
     * by derived ros nodes. It handles the communication with the scheduler
     * node.
     */
    class BaseNode : public rclcpp::Node
    {
      public:
        BaseNode(std::string node_name);
        virtual ~BaseNode() = default;
      private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Node* m_node;
        inline static adore_if_ros_scheduling::SchedulerNotificationManager *m_snm =
            0;               // object to coordinate communication with the scheduler
        bool useScheduler_;  // true, if scheduler is used
        double rate_;        // main rate of calling the functions in timers_ are called
        std::vector<std::function<void()>*> m_callbacks;

      public:
        /**
         * schedulerCallback - notifies scheduler of the new upper bound in time
         *
         */
        inline void schedulerCallback();
        /**
         * notifyNow - invoke notifyScheduler fcn manually
         *
         */
        void notifyNow();
        /**
         * init - initializes the ros node
         *
         */
        void init(int argc, char **argv, double rate, std::string nodename);
        /**
         * initSim - intilizes functionalites for simulation
         *
         */
        void initSim();
        /**
         * resume - resumes updating the upper time bound
         */
        void resume();
        /**
         * pause - pauses updating the upper time bound
         */
        void pause();
        /**
         * run
         */
        void run();
        /**
         * addTimerCallback - add a function that should be called periodically
         */
        void addTimerCallback(std::function<void()> *callbackFcn);
        /**
         * getParam - retrieve ros parameter
         */
    };
}  // namespace adore_if_ros_scheduling
}