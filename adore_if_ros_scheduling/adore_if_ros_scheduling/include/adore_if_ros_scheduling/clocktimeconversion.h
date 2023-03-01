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
 *   Matthias Nichting - initial implementation
 ********************************************************************************/

#include <rosgraph_msgs/Clock.h>

#include <utility>

namespace adore_if_ros_scheduling
{

    /**
     * This class convertes between between std::pair<uint32_t,uint32_t>  and
     * ROS message rosgraph_msgs/Clock
     */
    class ClockTimeConversion
    {
      public:
        void operator()(rosgraph_msgs::ClockConstPtr cl, std::pair<uint32_t, uint32_t> &value)
        {
            value.first = cl->clock.sec;
            value.second = cl->clock.nsec;
        }
        void operator()(const rosgraph_msgs::Clock &cl, std::pair<uint32_t, uint32_t> &value)
        {
            value.first = cl.clock.sec;
            value.second = cl.clock.nsec;
        }
        rosgraph_msgs::Clock operator()(const std::pair<uint32_t, uint32_t> &value)
        {
            rosgraph_msgs::Clock cl;
            cl.clock.sec = value.first;
            cl.clock.nsec = value.second;
            return cl;
        }
    };
}  // namespace adore_if_ros_scheduling
