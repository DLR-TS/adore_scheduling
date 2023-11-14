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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

#include "adore_scheduling_constants.h"
#include "schedulernotification.h"

namespace adore {
namespace scheduling {
/**
 * This class implements methods for communicating with a scheduling instance.
 */
class SchedulerNotificationManager {
 private:
  rclcpp::Publisher<adore_ros2_msgs::msg::SchedulerNotification>::SharedPtr
      m_notificationWriter;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_clientNameWriter;
  adore::scheduling::SchedulerNotification m_sn;
  std::chrono::nanoseconds m_duration;
  uint32_t m_duration_in_nanoseconds;  // in nanoseconds
  const uint32_t m_NANOS_PER_SECOND;
  bool m_pause;

 public:
  SchedulerNotificationManager(
      rclcpp::Node *node, unsigned int id,
      std::chrono::nanoseconds time_between_function_calls, bool reg = true);
  void publishClientName(std::string name);
  std::chrono::nanoseconds getDuration();
  uint32_t getDurationInNanoSeconds();
  void registerAtScheduler();
  void notifyScheduler(int64_t input_nanos);
  void pause();
  void resume();
};
}  // namespace scheduling
}  // namespace adore