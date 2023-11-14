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

#include <adore_basenode/schedulernotificationmanager.h>

namespace adore {
namespace scheduling {
/**
 * This class implements methods for communicating with a scheduling instance.
 */
SchedulerNotificationManager::SchedulerNotificationManager(
    rclcpp::Node *node, unsigned int id,
    std::chrono::nanoseconds time_between_function_calls, bool reg /* = true*/)
    : m_NANOS_PER_SECOND(1e9) {
  m_duration = time_between_function_calls;
  m_duration_in_nanoseconds = m_duration.count();
  m_clientNameWriter = node->create_publisher<std_msgs::msg::String>(
      adore::scheduling::TOPIC_NAME_CLIENT_NAME, 1);
  m_notificationWriter =
      node->create_publisher<adore_ros2_msgs::msg::SchedulerNotification>(
          adore::scheduling::TOPIC_NAME_SCHEDULER_NOTIFICATION, 1);
  while (!m_notificationWriter->get_subscription_count() ||
         !m_clientNameWriter->get_subscription_count()) {
    std::cout << "wait for the scheduler ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  m_sn.setID(id);
  std::cout << "scheduler id : " << id << std::endl;
  if (reg) {
    registerAtScheduler();
  }
}
void SchedulerNotificationManager::publishClientName(std::string name) {
  std_msgs::msg::String msg;
  msg.data = std::to_string(m_sn.getID()) + ":" + name;
  m_clientNameWriter->publish(msg);
}
std::chrono::nanoseconds SchedulerNotificationManager::getDuration() {
  return m_duration;
}
uint32_t SchedulerNotificationManager::getDurationInNanoSeconds() {
  return m_duration_in_nanoseconds;
}
void SchedulerNotificationManager::registerAtScheduler() {
  m_sn.setUpperTimeLimit(0, m_duration_in_nanoseconds);
  m_notificationWriter->publish(m_sn.getMessage());
}
void SchedulerNotificationManager::notifyScheduler(int64_t input_nanos) {
  uint32_t nsec = input_nanos % m_NANOS_PER_SECOND;
  uint32_t sec = input_nanos / m_NANOS_PER_SECOND;
  m_sn.setUpperTimeLimit(sec, nsec + m_duration_in_nanoseconds);
  if (!m_pause) {
    m_notificationWriter->publish(m_sn.getMessage());
  }
}
void SchedulerNotificationManager::pause() { m_pause = true; }
void SchedulerNotificationManager::resume() {
  m_pause = false;
  m_notificationWriter->publish(m_sn.getMessage());
}
}  // namespace scheduling
}  // namespace adore