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
    namespace scheduling{
    BaseNode::BaseNode(std::string node_name)
    : rclcpp::Node(node_name)
    {
        m_node = this;
    }

    
        inline void BaseNode::schedulerCallback()
        {
            for(auto it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
            {
                (**it)();
            }
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
        void BaseNode::init(int argc, char **argv, double rate, std::string nodename)
        {
            rate_ = rate;
           /* m_pN->param(adore::scheduling::PARAM_NAME_USE_SCHEDULER, useScheduler_, false);
            {
                ros::NodeHandle np("~");
                np.getParam("rate", rate_);
            }*/
            if (rate == 0.0) useScheduler_ = false;
        }
        /**
         * initSim - intilizes functionalites for simulation
         *
         */
        void BaseNode::initSim()
        {
            if (useScheduler_)
            {
                m_snm = new adore_if_ros_scheduling::SchedulerNotificationManager(
                    m_node, std::hash<std::string>{}(std::string(m_node->get_namespace()) + std::string(m_node->get_name())),
                    (uint32_t)(1e9 / rate_));
                timer_ = m_node->create_wall_timer(
                    std::chrono::duration<uint32_t,std::ratio<static_cast<long int>(1e9)>>(m_snm->getDurationInNanoSeconds()), std::bind(&BaseNode::schedulerCallback, this));
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
            //rclcpp::init(argc, argv);
            //rclcpp::spin(std::make_shared<MinimalPublisher>());
            rclcpp::shutdown();
        }
        /**
         * addTimerCallback - add a function that should be called periodically
         */
        void BaseNode::addTimerCallback(std::function<void()> *callbackFcn)
        {
            m_callbacks.push_back(callbackFcn);
        //inline static void func(std::function<void()> &callback);
        //    timers_.push_back(m_pN->createTimer(ros::Duration(1 / rate_ / rate_factor),
        //                                        std::bind(&func, callbackFcn, std::placeholders::_1)));
        }

     
        inline static void func(std::function<void()> &callback);
}  // namespace adore_if_ros_scheduling
}
