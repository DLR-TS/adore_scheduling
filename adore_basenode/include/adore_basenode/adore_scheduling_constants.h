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
 *   Matthias Nichting
 ********************************************************************************/

#pragma once

/**
 * this header provides constants used in the adore_if_ros_scheduling project
 */
namespace adore
{
    namespace scheduling{
    const char TOPIC_NAME_SIMULATION_TIME[] = "/SIM/utc";
    const char TOPIC_NAME_CLOCK_TIME[] = "/clock";
    const char TOPIC_NAME_CLIENT_NAME[] = "/SIM/scheduling_client_names";
    const char TOPIC_NAME_SCHEDULER_NOTIFICATION[] = "/SIM/scheduling";

    const char PARAM_NAME_USE_SCHEDULER[] = "/use_scheduler";
}}