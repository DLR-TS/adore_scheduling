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

#include <adore_basenode/adore_scheduling_constants.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <adore_ros2_msgs/msg/scheduler_notification.hpp>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace adore
{
    namespace scheduling
    {

        /**
         * @brief Scheduler is a class which provides functionality for stepped simulation
         *
         */

        struct TimeMsgComp
        {
            bool operator()(const builtin_interfaces::msg::Time &obj_left, const builtin_interfaces::msg::Time &obj_right) const;
        };
        class Scheduler : public rclcpp::Node
        {

            bool less_than(builtin_interfaces::msg::Time const &obj_left, builtin_interfaces::msg::Time const &obj_right);
            bool equals(builtin_interfaces::msg::Time const &obj_left, builtin_interfaces::msg::Time const &obj_right);
            using ScheduleMap = std::multimap<builtin_interfaces::msg::Time, uint64_t, TimeMsgComp>;

        private:
            rclcpp::Subscription<adore_ros2_msgs::msg::SchedulerNotification>::SharedPtr m_notificationReader;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_clientNameReader;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_simulationTimeWriter;
            rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_clockTimeWriter;
            bool m_pause;                // true if scheduling is paused
            int m_minRegisters;          // number of registers that is needed to automatically start the scheduling
            bool m_autostart;            // true if scheduling should start without keyboard input
            bool m_started;              // ture if scheduling has started
            int m_printIntervalS;        // interval of console output in seconds (measured in simulation time)
            bool m_limitSimulationSpeed; // true if simulation speed is limited
            std::chrono::system_clock::time_point m_lastWallTime;
            builtin_interfaces::msg::Time m_lastRosTime;
            ScheduleMap *m_schedule;
            builtin_interfaces::msg::Time m_now;
            rosgraph_msgs::msg::Clock m_now_clockmsg;
            std::pair<builtin_interfaces::msg::Time, std::chrono::system_clock::time_point> m_lastTimeSet;
            std::unordered_map<uint64_t, std::string> m_clientNames;
            TimeMsgComp m_timeMsgComp;

            /**
             * Search for the registree info in m_schedule and replace the associated time key
             *
             * If the registree info is not found, it is added to the m_schedule
             *
             * @param ri registree info
             * @param tk time key
             */
            void updateSchedule(uint64_t ri, builtin_interfaces::msg::Time tk);

        public:
            Scheduler(int minRegisters, bool autostart);
            builtin_interfaces::msg::Time getTimeMessage();

        public:
            /**
             * init
             */
            void init();

            /**
             *  update uppter time limit associated with certain client
             */
            void updateClientUpperTimeLimit(const adore_ros2_msgs::msg::SchedulerNotification &msg);

            /**
             * set new simulation and clock time
             */
            inline void setNewTime(bool incrementalIncrease = false);

            /**
             * pause and unpause the time
             */
            void togglePause();

            /**
             * limit the speed of simulation to the speed of ros::WallTime (inaccurate)
             */
            void limitSimulationSpeed();

            /**
             * publisch new time signal
             */
            inline void write();

            /**
             * get time differnce in nano seconds
             */
            uint32_t getTimeDiff(builtin_interfaces::msg::Time subtrahend, builtin_interfaces::msg::Time minuend);

            /**
             * print time information to std::cout
             */
            void printTime();

            /**
             * save name of client associated with id
             */
            void saveClientName(const std_msgs::msg::String &msg);

            /**
             * print information to std::cout
             */
            void printInfo();

            /**
             * start the scheduling
             */
            void start();
        };
    }
}