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
 *   Matthias Nichting
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#include <adore_basenode/adore_scheduling_constants.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

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
#include <adore_scheduler/adore_scheduler.h>

namespace adore
{
namespace scheduling{

        bool TimeMsgComp::operator()(const builtin_interfaces::msg::Time& obj_left,const builtin_interfaces::msg::Time& obj_right) const
        {
            if (obj_left.sec < obj_right.sec)
            {
                return true;
            }
            else if (obj_left.sec == obj_right.sec && obj_left.nanosec < obj_right.nanosec)
            {
                return true;
            }
            return false;
        }

        bool less_than(builtin_interfaces::msg::Time const& obj_left, builtin_interfaces::msg::Time const& obj_right)
        {
            if (obj_left.sec < obj_right.sec)
            {
                return true;
            }
            else if (obj_left.sec == obj_right.sec && obj_left.nanosec < obj_right.nanosec)
            {
                return true;
            }
            return false;
        }
        bool equals(builtin_interfaces::msg::Time const& obj_left, builtin_interfaces::msg::Time const& obj_right)
        {
            return obj_left.sec == obj_right.sec && obj_left.nanosec == obj_right.nanosec;
        }

        
        

    /**
     * @brief Scheduler is a class which provides functionality for stepped simulation
     *
     */
    
        void Scheduler::updateSchedule(uint64_t ri, builtin_interfaces::msg::Time cl)
        {
            for (auto it = m_schedule->begin(); it != m_schedule->end(); ++it)
            {
                if (it->second == ri)
                {
                    auto nh = m_schedule->extract(it);
                    nh.key() = cl;
                    m_schedule->insert(std::move(nh));
                    return;
                }
            }
            m_schedule->insert(std::make_pair(cl, ri));
            std::cout << ri << " has registered." << std::endl;
            write();
            --m_minRegisters;
        }

        Scheduler::Scheduler(int minRegisters, bool autostart)
        : rclcpp::Node("adore_scheduler_node")
        {
            m_schedule = new ScheduleMap();

            m_notificationReader =
                this->create_subscription<adore_ros2_msgs::msg::SchedulerNotification>(
                    adore::scheduling::TOPIC_NAME_SCHEDULER_NOTIFICATION, 1000,
                    std::bind(&Scheduler::updateClientUpperTimeLimit, this, std::placeholders::_1));
                
            m_clientNameReader = 
                this->create_subscription<std_msgs::msg::String>(
                    adore::scheduling::TOPIC_NAME_CLIENT_NAME, 1000,
                    std::bind(&Scheduler::saveClientName, this, std::placeholders::_1));
            m_simulationTimeWriter = 
                this->create_publisher<std_msgs::msg::Float64>(adore::scheduling::TOPIC_NAME_SIMULATION_TIME, 1);
            m_clockTimeWriter = 
                this->create_publisher<rosgraph_msgs::msg::Clock>(adore::scheduling::TOPIC_NAME_CLOCK_TIME, 1);
            m_now = getTimeMessage();
            m_now_clockmsg = rosgraph_msgs::msg::Clock();
            m_lastTimeSet.first = getTimeMessage();
            m_lastTimeSet.second = std::chrono::system_clock::now();
            m_pause = false;
            m_autostart = autostart;
            m_started = false;
            m_minRegisters = minRegisters;
            m_lastWallTime = std::chrono::system_clock::now();
            m_lastRosTime = getTimeMessage();
            m_limitSimulationSpeed = false;
            m_printIntervalS = 10;
            m_timeMsgComp = TimeMsgComp();
        }

        builtin_interfaces::msg::Time Scheduler::getTimeMessage()
        {
            builtin_interfaces::msg::Time t;
            t.sec = 0;
            t.nanosec = 0;
            return t;
        }
     

        /**
         * init
         */
        void Scheduler::init()
        {
            std::cout << std::endl
                      << "Type s to start" << std::endl;
        }

        /**
         *  update uppter time limit associated with certain client
         */
        void Scheduler::updateClientUpperTimeLimit(const adore_ros2_msgs::msg::SchedulerNotification &msg)
        {
            updateSchedule(msg.identifier, msg.upper_time_limit);
            if (!m_pause && m_started)
            {
                setNewTime();
            }
            else if (!m_pause && m_minRegisters < 1 && m_autostart)
            {
                std::cout << "scheduling started automatically" << std::endl;
                m_started = true;
                setNewTime();
            }
        }

        /**
         * set new simulation and clock time
         */
        inline void Scheduler::setNewTime(bool incrementalIncrease/* = false*/)
        {
            if (!m_schedule->empty())
            {
                auto upper_time_limit = m_schedule->begin()->first;
                /**
                 * incrementalIncrease must be used in order to trigger initial timer event in ros based python nodes
                 */
                if (incrementalIncrease)
                {
                    // todo_20231108
                    /*auto d = (upper_time_limit.second - m_now.second) / 1000;
                    while (upper_time_limit.second > m_now.second)
                    {
                        m_now.second += d;
                        write();
                    }
                    */
                }
                if (m_timeMsgComp(m_now, upper_time_limit))
                {
                    bool print = false;
                    if (upper_time_limit.sec > m_now.sec && upper_time_limit.sec % m_printIntervalS == 0)
                        print = true;
                    m_now = upper_time_limit;
                    if (print)
                        printTime();
                    write();
                }
            }
            else
            {
                m_now.sec = 0;
                m_now.nanosec = 0;
                write();
            }
        }

        /**
         * pause and unpause the time
         */
        void Scheduler::togglePause()
        {
            m_pause = !m_pause;
            std::cout << (m_pause ? "pause at " : "resume at ");
            printTime();
            if (!m_pause)
                setNewTime();
        }

        /**
         * limit the speed of simulation to the speed of ros::WallTime (inaccurate)
         */
        void Scheduler::limitSimulationSpeed()
        {
            m_limitSimulationSpeed = !m_limitSimulationSpeed;
            if (m_limitSimulationSpeed)
            {
                m_lastTimeSet.first = m_now;
                m_lastTimeSet.second = std::chrono::system_clock::now();
            }
            std::cout << "limiting simulation speed is now " << (m_limitSimulationSpeed ? "activated" : "deactivated")
                      << std::endl;
        }

        /**
         * publisch new time signal
         */
        inline void Scheduler::write()
        {
            if (m_limitSimulationSpeed)
            {
                std::this_thread::sleep_for(std::chrono::nanoseconds(getTimeDiff(m_lastTimeSet.first, m_now)));
                m_lastTimeSet.first = m_now;
                m_lastTimeSet.second = std::chrono::system_clock::now();
            }
            std_msgs::msg::Float64 simulationTimeOutput;
            simulationTimeOutput.data = static_cast<double>(m_now.sec) + (static_cast<double>(m_now.nanosec)) * 1e-9;
            m_simulationTimeWriter->publish(simulationTimeOutput);
            m_now_clockmsg.clock = m_now;
            m_clockTimeWriter->publish(m_now_clockmsg);
        }

        /**
         * get time differnce in nano seconds
         */
        uint32_t Scheduler::getTimeDiff(builtin_interfaces::msg::Time subtrahend, builtin_interfaces::msg::Time minuend)
        {
            return minuend.sec * 1e9 + minuend.nanosec - subtrahend.sec * 1e9 - subtrahend.nanosec;
        }

        /**
         * print time information to std::cout
         */
        void Scheduler::printTime()
        {
            std::chrono::system_clock::time_point newWallTime = std::chrono::system_clock::now();
            double speedfactor =
                (static_cast<double>(m_now.nanosec) + (static_cast<double>(m_now.sec)) * 1e-9 -
                 static_cast<double>(m_lastRosTime.nanosec) - (static_cast<double>(m_lastRosTime.sec)) * 1e-9) /
                (std::chrono::duration_cast<std::chrono::nanoseconds>(newWallTime - m_lastWallTime).count());
            std::cout << "time = " << m_now.sec << "." << std::setfill('0') << std::setw(9) << m_now.nanosec
                      << "; rel. simulation speed = " << speedfactor << std::endl;
            m_lastWallTime = newWallTime;
            m_lastRosTime = m_now;
        }

        /**
         * save name of client associated with id
         */
        void Scheduler::saveClientName(const std_msgs::msg::String &msg)
        {
            std::string clientName = msg.data;
            size_t pos;
            uint64_t id = std::stoul(clientName, &pos, 10);
            m_clientNames.insert(std::make_pair(id, clientName.substr(pos + 1)));
        }

        /**
         * print information to std::cout
         */
        void Scheduler::printInfo()
        {
            std::cout << "\nscheduling info at ";
            printTime();
            for (auto i = m_schedule->begin(); i != m_schedule->end(); ++i)
            {
                auto clientname_it = m_clientNames.find(i->second);
                std::string clientname = clientname_it != m_clientNames.end() ? " : " + clientname_it->second : "";
                std::cout << "  max time for id " << i->second << " is " << i->first.sec << "." << std::setfill('0')
                          << std::setw(9) << i->first.nanosec << clientname << std::endl;
            }
        }

        /**
         * start the scheduling
         */
        void Scheduler::start()
        {
            std::cout << "scheduling started" << std::endl;
            m_started = true;
            setNewTime(true);
        }
} // namespace adore_if_ros_scheduling

}
int main()
{
    return 0;
}