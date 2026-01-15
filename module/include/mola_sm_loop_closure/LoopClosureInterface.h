// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
// Licensed under the GNU GPL v3.
//
// This file is part of MOLA.
// MOLA is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// MOLA. If not, see <https://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>

namespace mola
{
class LoopClosureInterface : public mrpt::rtti::CObject, public mrpt::system::COutputLogger
{
    DEFINE_VIRTUAL_MRPT_OBJECT(LoopClosureInterface, mola)

   public:
    LoopClosureInterface();
    virtual ~LoopClosureInterface();

    // Disable copy and move operations
    LoopClosureInterface(const LoopClosureInterface&)            = delete;
    LoopClosureInterface& operator=(const LoopClosureInterface&) = delete;
    LoopClosureInterface(LoopClosureInterface&&)                 = delete;
    LoopClosureInterface& operator=(LoopClosureInterface&&)      = delete;

    /** @name Main API
     * @{ */

    virtual void initialize(const mrpt::containers::yaml& cfg) = 0;

    /** Find and apply loop closures in the input/output simplemap */
    virtual void process(mrpt::maps::CSimpleMap& sm) = 0;
};

}  // namespace mola