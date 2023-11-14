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

#include <adore_basenode/schedulernotification.h>

namespace adore {
namespace scheduling {
/**
 * This class holds data suitable for communication with a scheduling instance.
 */
SchedulerNotification::SchedulerNotification() {
  m_msg = std::make_shared<adore_ros2_msgs::msg::SchedulerNotification>();
}
uint32_t SchedulerNotification::getUpperTimeLimitSec() const {
  return m_upperTimeLimitSec;
}
uint32_t SchedulerNotification::getUpperTimeLimitNsec() const {
  return m_upperTimeLimitNsec;
}
std::pair<uint32_t, uint32_t> SchedulerNotification::getUpperTimeLimitPair()
    const {
  return std::make_pair(m_upperTimeLimitSec, m_upperTimeLimitNsec);
}
void SchedulerNotification::setUpperTimeLimit(uint32_t sec, uint32_t nsec) {
  while (nsec >= 1e9) {
    nsec -= 1e9;
    sec += 1;
  }
  m_upperTimeLimitSec = sec;
  m_upperTimeLimitNsec = nsec;
}
void SchedulerNotification::setZero(unsigned int id) {
  m_msg->identifier = id;
  m_upperTimeLimitSec = 0;
  m_upperTimeLimitNsec = 0;
}
unsigned int SchedulerNotification::getID() const { return m_msg->identifier; }
void SchedulerNotification::setID(unsigned int id) { m_msg->identifier = id; }
std::unique_ptr<adore_ros2_msgs::msg::SchedulerNotification>
SchedulerNotification::getMessage() {
  auto msg = std::make_unique<adore_ros2_msgs::msg::SchedulerNotification>();
  msg->identifier = m_msg->identifier;
  msg->upper_time_limit.sec = m_upperTimeLimitSec;
  msg->upper_time_limit.nanosec = m_upperTimeLimitNsec;
  return msg;
}
}  // namespace scheduling
}  // namespace adore