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

#include <adore_ros2_msgs/msg/scheduler_notification.hpp>
#include <cstdint>
#include <utility>

namespace adore {
namespace scheduling {
/**
 * This class holds data suitable for communication with a scheduling instance.
 */
class SchedulerNotification {
  uint32_t m_upperTimeLimitSec;   // seconds
  uint32_t m_upperTimeLimitNsec;  // nanosecons
  std::shared_ptr<adore_ros2_msgs::msg::SchedulerNotification> m_msg;

 public:
  SchedulerNotification();
  uint32_t getUpperTimeLimitSec() const;
  uint32_t getUpperTimeLimitNsec() const;
  std::pair<uint32_t, uint32_t> getUpperTimeLimitPair() const;
  void setUpperTimeLimit(uint32_t sec, uint32_t nsec);
  void setZero(unsigned int id);
  unsigned int getID() const;
  void setID(unsigned int id);
  std::unique_ptr<adore_ros2_msgs::msg::SchedulerNotification> getMessage();
};
}  // namespace scheduling
}  // namespace adore