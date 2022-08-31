/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR).
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

#include <adore_scheduling/schedulernotification.h>
#include <adore_if_ros_scheduling/SchedulerNotification.h>

namespace adore_if_ros_scheduling
{
    /**
     * This class convertes between adore_scheduling::SchedulerNotification and ROS message adore_if_ros_scheduling::SchedulerNotification.
     */
    class SchedulerNotificationConversion
    {
    public:
        adore_scheduling::SchedulerNotification operator()(adore_if_ros_scheduling::SchedulerNotificationConstPtr msg)
        {
            return adore_scheduling::SchedulerNotification(msg->identifier, msg->upperTimeLimit.sec, msg->upperTimeLimit.nsec);
        }
        adore_if_ros_scheduling::SchedulerNotification operator()(const adore_scheduling::SchedulerNotification &sn)
        {
            adore_if_ros_scheduling::SchedulerNotification msg;
            msg.upperTimeLimit.sec = sn.getUpperTimeLimitSec();
            msg.upperTimeLimit.nsec = sn.getUpperTimeLimitNsec();
            msg.identifier = sn.getID();
            return msg;
        }
    };
}