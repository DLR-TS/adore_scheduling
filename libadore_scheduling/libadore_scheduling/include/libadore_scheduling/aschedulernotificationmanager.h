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

#include <libadore_scheduling/schedulernotification.h>
#include <string>

namespace adore_scheduling
{
    /**
     * This is an abstract class for managing the communication with a scheduling instance.
     */
    class ASchedulerNotificationManager
    {
    protected:
        adore_scheduling::SchedulerNotification m_sn;
        uint32_t m_duration; // in nanoseconds
        bool m_pause;

    public:
        ASchedulerNotificationManager(uint32_t duration)
            : m_duration(duration), m_pause(false)
        {
        }
        virtual void publishClientName(std::string name) = 0;
        virtual void registerAtScheduler() = 0;
        virtual void notifyScheduler(uint32_t sec, uint32_t nsec) = 0;
        virtual void pause() = 0;
        virtual void resume() = 0;
    };
}
