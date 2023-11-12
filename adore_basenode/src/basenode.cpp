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

namespace adore
{
    namespace scheduling
    {
        BaseNode::BaseNode(std::chrono::nanoseconds duration_between_function_calls, std::string node_name)
            : rclcpp::Node(node_name), useScheduler_(true)
        {
            init(duration_between_function_calls);
        }

        inline void BaseNode::schedulerCallback()
        {
            for (auto it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
            {
                if (it->first.first != 1)
                {
                    if (it->first.second == 0)
                    {
                        it->first.second = it->first.first - 1;
                    }
                    else
                    {
                        --it->first.second;
                        continue;
                    }
                }
                (*(it->second))();
                //(**it)();
            }
            if (useScheduler_)
                m_snm->notifyScheduler(m_node->now().nanoseconds());
        }
        /**
         * notifyNow - invoke notifyScheduler fcn manually
         *
         */
        void BaseNode::notifyNow()
        {
            m_snm->notifyScheduler(m_node->now().nanoseconds());
        }
        /**
         * init - initializes the ros node
         *
         */
        void BaseNode::init(std::chrono::nanoseconds duration_between_function_calls)
        {
            m_node = this;

            m_time_between_function_calls = std::chrono::duration_cast<std::chrono::nanoseconds>(duration_between_function_calls);
            /* m_pN->param(adore::scheduling::PARAM_NAME_USE_SCHEDULER, useScheduler_, false);
            {
                ros::NodeHandle np("~");
                np.getParam("rate", rate_);
            }*/
            auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
            timer_ = rclcpp::create_timer(m_node, this->get_clock(), m_time_between_function_calls, std::bind(&BaseNode::schedulerCallback, this));

            // timer_ = m_node->create_wall_timer(
            //         m_time_between_function_calls, std::bind(&BaseNode::schedulerCallback, this));
            if (m_time_between_function_calls.count() == 0)
                useScheduler_ = false;
            if (useScheduler_)
            {
                m_snm = new adore::scheduling::SchedulerNotificationManager(
                    m_node, std::hash<std::string>{}(std::string(m_node->get_namespace()) + std::string(m_node->get_name())),
                    m_time_between_function_calls);
                m_snm->publishClientName(std::string(m_node->get_namespace()) + std::string(m_node->get_name()));
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
         * run
         */
        void BaseNode::run()
        {
            // rclcpp::init(argc, argv);
            // rclcpp::spin(std::make_shared<MinimalPublisher>());
            rclcpp::shutdown();
        }
        /**
         * addTimerCallback - add a function that should be called periodically
         */
        void BaseNode::addTimerCallback(std::shared_ptr<std::function<void()>> callbackFcn, unsigned int frequency_divisor)
        {
            m_callbacks.push_back(std::make_pair(std::make_pair(frequency_divisor, frequency_divisor - 1), callbackFcn));

        }

    } // namespace adore_if_ros_scheduling
}
