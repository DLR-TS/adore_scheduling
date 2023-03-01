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

#include <adore_if_ros_scheduling/schedulernotificationmanager.h>

#include <string>
#include <vector>

namespace adore_if_ros_scheduling
{

    /**
     *  Base class for ros nodes - Baseapp provides functions that can be used
     * by derived ros nodes. It handles the communication with the scheduler
     * node.
     */
    class Baseapp
    {
      public:
        Baseapp() {}

      private:
        inline static ros::NodeHandle *m_pN = 0;
        std::vector<ros::Timer> timers_;  // functions that are periodically called
        inline static adore_if_ros_scheduling::SchedulerNotificationManager *snm_ =
            0;               // object to coordinate communication with the scheduler
        bool useScheduler_;  // true, if scheduler is used
        double rate_;        // main rate of calling the functions in timers_ are called

      public:
        /**
         * schedulerCallback - notifies scheduler of the new upper bound in time
         *
         */
        inline static void schedulerCallback(const ros::TimerEvent &e)
        {
            snm_->notifyScheduler(ros::Time::now().sec, ros::Time::now().nsec);
        }
        /**
         * init - initializes the ros node
         *
         */
        void init(int argc, char **argv, double rate, std::string nodename)
        {
            rate_ = rate;
            ros::init(argc, argv, nodename);
            ros::NodeHandle *n = new ros::NodeHandle();
            m_pN = n;
            m_pN->param(adore_if_ros_scheduling::PARAM_NAME_USE_SCHEDULER, useScheduler_, false);
            {
                ros::NodeHandle np("~");
                np.getParam("rate", rate_);
            }
            if (rate == 0.0) useScheduler_ = false;
        }
        /**
         * initSim - intilizes functionalites for simulation
         *
         */
        void initSim()
        {
            if (useScheduler_)
            {
                snm_ = new adore_if_ros_scheduling::SchedulerNotificationManager(
                    m_pN, std::hash<std::string>{}(ros::this_node::getNamespace() + ros::this_node::getName()),
                    (uint32_t)(1e9 / rate_));
                timers_.push_back(m_pN->createTimer(ros::Duration(1 / rate_), schedulerCallback));
                snm_->publishClientName(ros::this_node::getName());
            }
        }
        /**
         * getRosNodeHandle - return ros::NodeHandle pointer
         */
        static ros::NodeHandle *getRosNodeHandle() { return m_pN; }
        /**
         * resume - resumes updating the upper time bound
         */
        virtual void resume() { snm_->resume(); }
        /**
         * pause - pauses updating the upper time bound
         */
        virtual void pause() { snm_->pause(); }
        /**
         * run
         */
        virtual void run()
        {
            while (m_pN->ok())
            {
                ros::spin();
            }
        }
        inline static void func(std::function<void()> &callback, const ros::TimerEvent &te) { callback(); }
        /**
         * addTimerCallback - add a function that should be called periodically
         */
        virtual void addTimerCallback(std::function<void()> &callbackFcn, double rate_factor = 1.0)
        {
            timers_.push_back(m_pN->createTimer(ros::Duration(1 / rate_ / rate_factor),
                                                std::bind(&func, callbackFcn, std::placeholders::_1)));
        }

        /**
         * getParam - retrieve ros parameter
         */
        template <typename T>
        bool getParam(const std::string name, T &val)
        {
            if (name.compare(0, 1, "~") == 0)
            {
                ros::NodeHandle np("~");
                return np.getParam(name.substr(1), val);
            }
            return m_pN->getParam(name, val);
        }
        /**
         * getParam - retrieve ros parameter with default
         */
        template <typename T>
        bool getParam(const std::string name, T &val, const T &default_val)
        {
            return m_pN->param<T>(name, val, default_val);
        }
    };
}  // namespace adore_if_ros_scheduling
