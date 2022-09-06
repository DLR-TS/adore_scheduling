/********************************************************************************
 * Copyright (C) 2017-2022 German Aerospace Center (DLR).
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

#include <lib_adore_if_ros_scheduling/adore_if_ros_scheduling_constants.h>
#include <lib_adore_if_ros_scheduling/clocktimeconversion.h>
#include <lib_adore_if_ros_scheduling/schedulernotificationconversion.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

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

namespace adore_if_ros_scheduling
{

    template <typename RegistreeInfo, typename TimeKeyType>
    /**
     * @brief Scheduler is a class which provides functionality for stepped simulation
     *
     */
    class Scheduler
    {
        using ScheduleMap = std::multimap<TimeKeyType, RegistreeInfo>;

      private:
        ros::Subscriber m_notificationReader;
        ros::Subscriber m_clientNameReader;
        ros::Publisher m_simulationTimeWriter;
        ros::Publisher m_clockTimeWriter;
        SchedulerNotificationConversion m_schedulerNotificationConversion;
        ClockTimeConversion m_clockTimeConversion;
        bool m_pause;          // true, if scheduling is paused
        int m_minRegisters;    // number of registers that is needed to start the scheduling
        bool m_autostart;      // true, if scheduling should start without keyboard input
        bool m_started;        // ture, if scheduling has started
        int m_printIntervalS;  // interval of console output in second (measured in simulation time)
        bool m_limitSimulationSpeed;
        std::pair<uint32_t, uint32_t> m_lastWallTime;
        std::pair<uint32_t, uint32_t> m_lastRosTime;
        ScheduleMap *m_schedule;
        TimeKeyType m_now;
        std::pair<TimeKeyType, std::pair<uint32_t, uint32_t>> m_lastTimeSet;
        std::unordered_map<RegistreeInfo, std::string> m_clientNames;

        /**
         * Search for the registree info in m_schedule and replace the associated time key
         *
         * If the registree info is not found, it is added to the m_schedule
         *
         * @param ri registree info
         * @param tk time key
         */
        void updateSchedule(RegistreeInfo ri, TimeKeyType tk)
        {
            for (auto it = m_schedule->begin(); it != m_schedule->end(); ++it)
            {
                if (it->second == ri)
                {
                    auto nh = m_schedule->extract(it);
                    nh.key() = tk;
                    m_schedule->insert(std::move(nh));
                    return;
                }
            }
            m_schedule->insert(std::make_pair(tk, ri));
            std::cout << ri << " has registered." << std::endl;
            write();
            --m_minRegisters;
        }

        Scheduler(ros::NodeHandle &n, int minRegisters, bool autostart, bool tcp_no_delay)
        {
            m_schedule = new ScheduleMap();
            m_notificationReader =
                n.subscribe(TOPIC_NAME_SCHEDULIER_NOTIFICATION, 1000, &Scheduler::updateClientUpperTimeLimit, this,
                            ros::TransportHints().tcpNoDelay(tcp_no_delay));
            m_clientNameReader = n.subscribe(TOPIC_NAME_CLIENT_NAME, 1000, &Scheduler::saveClientName, this,
                                             ros::TransportHints().tcpNoDelay(tcp_no_delay));
            m_simulationTimeWriter = n.advertise<std_msgs::Float64>(TOPIC_NAME_SIMULATION_TIME, 1);
            m_clockTimeWriter = n.advertise<rosgraph_msgs::Clock>(TOPIC_NAME_CLOCK_TIME, 1);
            m_now = std::make_pair(0, 0);
            m_lastTimeSet.first = std::make_pair(0, 0);
            m_lastTimeSet.second = std::make_pair(0, 0);
            m_pause = false;
            m_autostart = autostart;
            m_started = false;
            m_minRegisters = minRegisters;
            m_lastWallTime = std::make_pair(ros::WallTime::now().sec, ros::WallTime::now().nsec);
            m_lastRosTime = std::make_pair(ros::Time::now().sec, ros::Time::now().nsec);
            m_limitSimulationSpeed = false;
            m_printIntervalS = 10;
        }

      public:
        /**
         * return instance of Scheduler class
         */
        static Scheduler *getInstance(ros::NodeHandle &n, int minRegisters, bool autostart, bool tcp_no_delay)
        {
            return new Scheduler(n, minRegisters, autostart, tcp_no_delay);
        }
        /**
         * init
         */
        void init() { std::cout << std::endl << "Type s to start" << std::endl; }
        /**
         *  update uppter time limit associated with certain client
         */
        void updateClientUpperTimeLimit(const adore_if_ros_scheduling_msg::SchedulerNotification::ConstPtr &msg)
        {
            adore_scheduling::SchedulerNotification clientNotification = m_schedulerNotificationConversion(msg);
            updateSchedule(clientNotification.getID(), clientNotification.getUpperTimeLimitPair());
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
        inline void setNewTime(bool incrementalIncrease = false)
        {
            if (!m_schedule->empty())
            {
                auto upper_time_limit = m_schedule->begin()->first;
                /**
                 * incrementalIncrease must be used in order to trigger initial timer event in ros based python nodes
                 */
                if (incrementalIncrease)
                {
                    auto d = (upper_time_limit.second - m_now.second) / 1000;
                    while (upper_time_limit.second > m_now.second)
                    {
                        m_now.second += d;
                        write();
                    }
                }
                if (m_now < upper_time_limit)
                {
                    bool print = false;
                    if (upper_time_limit.first > m_now.first && upper_time_limit.first % m_printIntervalS == 0)
                        print = true;
                    m_now = upper_time_limit;
                    if (print) printTime();
                    write();
                }
            }
            else
            {
                m_now.first = 0;
                m_now.second = 0;
                write();
            }
        }
        /**
         * pause and unpause the time
         */
        void togglePause()
        {
            m_pause = !m_pause;
            std::cout << (m_pause ? "pause at " : "resume at ");
            printTime();
            if (!m_pause) setNewTime();
        }
        /**
         * limit the speed of simulation to the speed of ros::WallTime (inaccurate)
         */
        void limitSimulationSpeed()
        {
            m_limitSimulationSpeed = !m_limitSimulationSpeed;
            if (m_limitSimulationSpeed)
            {
                m_lastTimeSet.first = m_now;
                m_lastTimeSet.second = std::make_pair(ros::WallTime::now().sec, ros::WallTime::now().nsec);
            }
            std::cout << "limiting simulation speed is now " << (m_limitSimulationSpeed ? "activated" : "deactivated")
                      << std::endl;
        }
        /**
         * publisch new time signal
         */
        inline void write()
        {
            if (m_limitSimulationSpeed)
            {
                std::this_thread::sleep_for(std::chrono::nanoseconds(getTimeDiff(m_lastTimeSet.first, m_now)));
                m_lastTimeSet.first = m_now;
                m_lastTimeSet.second = std::make_pair(ros::WallTime::now().sec, ros::WallTime::now().nsec);
            }
            std_msgs::Float64 simulationTimeOutput;
            simulationTimeOutput.data = static_cast<double>(m_now.first) + (static_cast<double>(m_now.second)) * 1e-9;
            m_simulationTimeWriter.publish(simulationTimeOutput);
            m_clockTimeWriter.publish(m_clockTimeConversion(m_now));
        }
        /**
         * get time differnce in nano seconds
         */
        uint32_t getTimeDiff(TimeKeyType subtrahend, TimeKeyType minuend)
        {
            return minuend.first * 1e9 + minuend.second - subtrahend.first * 1e9 - subtrahend.second;
        }
        /**
         * print time information to std::cout
         */
        void printTime()
        {
            std::pair<uint32_t, uint32_t> newWallTime =
                std::make_pair(ros::WallTime::now().sec, ros::WallTime::now().nsec);
            double speedfactor =
                (static_cast<double>(m_now.first) + (static_cast<double>(m_now.second)) * 1e-9 -
                 static_cast<double>(m_lastRosTime.first) - (static_cast<double>(m_lastRosTime.second)) * 1e-9) /
                (static_cast<double>(newWallTime.first) + (static_cast<double>(newWallTime.second)) * 1e-9 -
                 static_cast<double>(m_lastWallTime.first) - (static_cast<double>(m_lastWallTime.second)) * 1e-9);
            std::cout << "time = " << m_now.first << "." << std::setfill('0') << std::setw(9) << m_now.second
                      << "; rel. simulation speed = " << speedfactor << std::endl;
            m_lastWallTime = newWallTime;
            m_lastRosTime = m_now;
        }
        /**
         * save name of client associated with id
         */
        void saveClientName(const std_msgs::String::ConstPtr &msg)
        {
            std::string clientName = msg->data;
            size_t pos;
            RegistreeInfo id = std::stoul(clientName, &pos, 10);
            m_clientNames.insert(std::make_pair(id, clientName.substr(pos + 1)));
        }
        /**
         * print information to std::cout
         */
        void printInfo()
        {
            std::cout << "\nscheduling info at ";
            printTime();
            for (auto i = m_schedule->begin(); i != m_schedule->end(); ++i)
            {
                auto clientname_it = m_clientNames.find(i->second);
                std::string clientname = clientname_it != m_clientNames.end() ? " : " + clientname_it->second : "";
                std::cout << "  max time for id " << i->second << " is " << i->first.first << "." << std::setfill('0')
                          << std::setw(9) << i->first.second << clientname << std::endl;
            }
        }
        /**
         * start the scheduling
         */
        void start()
        {
            std::cout << "scheduling started" << std::endl;
            m_started = true;
            setNewTime(true);
        }
    };
}  // namespace adore_if_ros_scheduling
