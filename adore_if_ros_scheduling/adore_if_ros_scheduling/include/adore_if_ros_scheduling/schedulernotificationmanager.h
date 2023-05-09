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

#include <adore_if_ros_scheduling/adore_if_ros_scheduling_constants.h>
#include <adore_if_ros_scheduling/schedulernotificationconversion.h>
#include <libadore_scheduling/aschedulernotificationmanager.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <thread>

namespace adore_if_ros_scheduling
{
    /**
     * This class implements methods for communicating with a scheduling instance.
     */
    class SchedulerNotificationManager : adore_scheduling::ASchedulerNotificationManager
    {
      private:
        ros::NodeHandle *m_pN;
        ros::Publisher m_notificationWriter;
        ros::Publisher m_clientNameWriter;
        SchedulerNotificationConversion m_schedulerNotificationConversion;

      public:
        SchedulerNotificationManager(ros::NodeHandle *n, unsigned int id, uint32_t duration, bool reg = true)
            : adore_scheduling::ASchedulerNotificationManager(duration), m_pN(n)
        {
            m_clientNameWriter = m_pN->advertise<std_msgs::String>(adore_if_ros_scheduling::TOPIC_NAME_CLIENT_NAME, 1);
            m_notificationWriter =
                m_pN->advertise<adore_if_ros_scheduling_msg::SchedulerNotification>(adore_if_ros_scheduling::TOPIC_NAME_SCHEDULER_NOTIFICATION, 1);
            while (!m_notificationWriter.getNumSubscribers())
            {
                std::cout << "wait for the scheduler ..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            m_sn.setID(id);
            std::cout << "scheduler id : " << id << std::endl;
            if (reg) registerAtScheduler();
        }
        void publishClientName(std::string name) override
        {
            std_msgs::String msg;
            msg.data = std::to_string(m_sn.getID()) + ":" + name;
            m_clientNameWriter.publish(msg);
        }
        void registerAtScheduler() override
        {
            m_sn.setUpperTimeLimit(0, m_duration);
            m_notificationWriter.publish(m_schedulerNotificationConversion(m_sn));
        }
        void notifyScheduler(uint32_t sec, uint32_t nsec) override
        {
            m_sn.setUpperTimeLimit(sec, nsec + m_duration);
            if (!m_pause)
            {
                m_notificationWriter.publish(m_schedulerNotificationConversion(m_sn));
            }
        }
        void pause() override { m_pause = true; }
        void resume() override
        {
            m_pause = false;
            m_notificationWriter.publish(m_schedulerNotificationConversion(m_sn));
        }
    };
}  // namespace adore_if_ros_scheduling
