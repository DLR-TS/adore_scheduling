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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#include <adore_basenode/basenode.h>

namespace adore {
namespace scheduling {
/**
 * constructor - duration_between_function_calls is the reciprocal of the main
 * frequency of the node.
 */
BaseNode::BaseNode(std::chrono::nanoseconds duration_between_function_calls,
                   std::string node_name)
    : rclcpp::Node(node_name), m_use_scheduler(false) {
  bool use_sim_time = false;
  this->get_parameter("use_sim_time", use_sim_time);
  this->declare_parameter(adore::scheduling::PARAM_NAME_USE_SCHEDULER,
                          m_use_scheduler);
  this->get_parameter(adore::scheduling::PARAM_NAME_USE_SCHEDULER,
                      m_use_scheduler);
  if (m_use_scheduler && !use_sim_time) {
    std::cout << std::endl
              << "adore::scheduling::BaseNode: parameter use_sim_time for node "
              << node_name
              << "is False while parameter use_scheduler is set to True. If "
                 "use_scheduler is set to True, use_sim_time must be set to "
                 "True, too."
              << std::endl;
    throw std::logic_error(
        "Parameter issue: use_scheduler is True while use_sim_time is not "
        "True.");
  }
  init(duration_between_function_calls);
}
/**
 * mainCallback
 */
inline void BaseNode::mainCallback() {
  for (auto it = m_callbacks.begin(); it != m_callbacks.end(); ++it) {
    if (it->first.first != 1) {
      // frequency divisor defined for this callback
      if (it->first.second == 0) {
        // this iteration, callback is called; set counter back
        it->first.second = it->first.first - 1;
      } else {
        // this iteration, callback is not called; decrement counter
        --it->first.second;
        continue;
      }
    }
    // call callback
    (*(it->second))();
  }
  // notify callback, if in scheduling mode
  if (m_use_scheduler) {
    m_snm->notifyScheduler(m_node->now().nanoseconds());
  }
}
/**
 * notifyNow - invoke notifyScheduler fcn manually
 *
 */
void BaseNode::notifyNow() {
  m_snm->notifyScheduler(m_node->now().nanoseconds());
}
/**
 * init - initializes the ros node
 *
 */
void BaseNode::init(std::chrono::nanoseconds duration_between_function_calls) {
  m_node = this;
  m_time_between_function_calls =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          duration_between_function_calls);
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  m_timer = rclcpp::create_timer(m_node, m_node->get_clock(),
                                 m_time_between_function_calls,
                                 std::bind(&BaseNode::mainCallback, this));
  if (m_time_between_function_calls.count() == 0) {
    m_use_scheduler = false;
  }
  if (m_use_scheduler) {
    m_snm = new adore::scheduling::SchedulerNotificationManager(
        m_node,
        std::hash<std::string>{}(std::string(m_node->get_namespace()) +
                                 std::string(m_node->get_name())),
        m_time_between_function_calls);
    m_snm->publishClientName(std::string(m_node->get_namespace()) +
                             std::string(m_node->get_name()));
  }
}
/**
 * resume - resumes updating the upper time bound
 */
void BaseNode::resume() { m_snm->resume(); }
/**
 * pause - pauses updating the upper time bound
 */
void BaseNode::pause() { m_snm->pause(); }
/**
 * addTimerCallback - add a function that should be called periodically
 */
void BaseNode::addTimerCallback(
    std::shared_ptr<std::function<void()>> callbackFcn,
    unsigned int frequency_divisor) {
  m_callbacks.push_back(std::make_pair(
      std::make_pair(frequency_divisor, frequency_divisor - 1), callbackFcn));
}

}  // namespace scheduling
}  // namespace adore
