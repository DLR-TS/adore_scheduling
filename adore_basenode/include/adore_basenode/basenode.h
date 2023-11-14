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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "schedulernotificationmanager.h"

namespace adore {
namespace scheduling {
/**
 *  Base class for ros nodes - BaseNode provides functions that can be used
 * by derived ros nodes. It handles the communication with the scheduler
 * node.
 */
class BaseNode : public rclcpp::Node {
 public:
  BaseNode(std::chrono::nanoseconds duration_between_function_calls,
           std::string nodename);
  virtual ~BaseNode() = default;

 private:
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Node *m_node;
  inline static adore::scheduling::SchedulerNotificationManager *m_snm =
      0;  // object to coordinate communication with the scheduler
  bool m_use_scheduler;  // true, if scheduler is used
  std::chrono::nanoseconds
      m_time_between_function_calls;  // main rate of calling the functions in
                                      // timers_ are called
  // m_callbacks: pair < pair < frequency divisor,
  // skip_next_x_iterations_before_next_call>, fcn ptr>
  std::vector<std::pair<std::pair<unsigned int, unsigned int>,
                        std::shared_ptr<std::function<void()>>>>
      m_callbacks;

 public:
  /**
   * mainCallback - calls all callbacks added via addTimerCallback and notifies
   * scheduler of the new upper bound in time
   *
   */
  inline void mainCallback();
  /**
   * notifyNow - invoke notifyScheduler fcn manually
   *
   */
  void notifyNow();
  /**
   * init - initializes the ros node
   *
   */
  void init(std::chrono::nanoseconds duration_between_function_calls);
  /**
   * resume - resumes updating the upper time bound
   */
  void resume();
  /**
   * pause - pauses updating the upper time bound
   */
  void pause();
  /**
   * addTimerCallback - add a function that should be called periodically
   */
  void addTimerCallback(std::shared_ptr<std::function<void()>> callbackFcn,
                        unsigned int frequency_divisor = 1);
};
}  // namespace scheduling
}  // namespace adore
