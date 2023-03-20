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

#include <cstdint>
#include <utility>

namespace adore_scheduling
{
    /**
     * This class holds data suitable for communication with a scheduling instance.
     */
    class SchedulerNotification
    {
        unsigned int m_identifier;      // identifier that is used by the scheduler to match messages with ros nodes
        uint32_t m_upperTimeLimitSec;   // seconds
        uint32_t m_upperTimeLimitNsec;  // nanosecons

    public:
        SchedulerNotification() {}
        SchedulerNotification(unsigned int id, uint32_t upperSec, uint32_t upperNsec) : m_identifier(id), m_upperTimeLimitSec(upperSec), m_upperTimeLimitNsec(upperNsec)
        {
        }
        uint32_t getUpperTimeLimitSec() const
        {
            return m_upperTimeLimitSec;
        }
        uint32_t getUpperTimeLimitNsec() const
        {
            return m_upperTimeLimitNsec;
        }
        std::pair<uint32_t, uint32_t> getUpperTimeLimitPair() const
        {
            return std::make_pair(m_upperTimeLimitSec, m_upperTimeLimitNsec);
        }
        void setUpperTimeLimit(uint32_t sec, uint32_t nsec)
        {
            while (nsec >= 1e9)
            {
                nsec -= 1e9;
                sec += 1;
            }
            m_upperTimeLimitSec = sec;
            m_upperTimeLimitNsec = nsec;
        }
        void setZero(unsigned int id)
        {
            m_identifier = id;
            m_upperTimeLimitSec = 0;
            m_upperTimeLimitNsec = 0;
        }
        unsigned int getID() const
        {
            return m_identifier;
        }
        void setID(unsigned int id)
        {
            m_identifier = id;
        }
    };
}