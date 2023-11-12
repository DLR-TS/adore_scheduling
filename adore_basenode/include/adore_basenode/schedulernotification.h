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

#pragma once

#include <cstdint>
#include <utility>
#include <adore_ros2_msgs/msg/scheduler_notification.hpp>

namespace adore
{
    namespace scheduling
    {
        /**
         * This class holds data suitable for communication with a scheduling instance.
         */
        class SchedulerNotification
        {
            uint32_t m_upperTimeLimitSec;  // seconds
            uint32_t m_upperTimeLimitNsec; // nanosecons
            std::shared_ptr<adore_ros2_msgs::msg::SchedulerNotification> m_msg;

        public:
            SchedulerNotification()
            {
                m_msg = std::make_shared<adore_ros2_msgs::msg::SchedulerNotification>();
            }
            uint32_t getUpperTimeLimitSec() const
            {
                return m_upperTimeLimitSec;
            }
            uint32_t getUpperTimeLimitNsec() const
            {
                return m_upperTimeLimitNsec;
            }
            std::pair<uint32_t, uint32_t> getUpperTimeLimitPair() const
            {
                return std::make_pair(m_upperTimeLimitSec, m_upperTimeLimitNsec);
            }
            void setUpperTimeLimit(uint32_t sec, uint32_t nsec)
            {
                while (nsec >= 1e9)
                {
                    nsec -= 1e9;
                    sec += 1;
                }
                m_upperTimeLimitSec = sec;
                m_upperTimeLimitNsec = nsec;
            }
            void setZero(unsigned int id)
            {
                m_msg->identifier = id;
                m_upperTimeLimitSec = 0;
                m_upperTimeLimitNsec = 0;
            }
            unsigned int getID() const
            {
                return m_msg->identifier;
            }
            void setID(unsigned int id)
            {
                m_msg->identifier = id;
            }
            std::unique_ptr<adore_ros2_msgs::msg::SchedulerNotification> getMessage()
            {
                auto msg = std::make_unique<adore_ros2_msgs::msg::SchedulerNotification>();
                msg->identifier = m_msg->identifier;
                msg->upper_time_limit.sec = m_upperTimeLimitSec;
                msg->upper_time_limit.nanosec = m_upperTimeLimitNsec;
                return msg;
            }
        };
    }
}