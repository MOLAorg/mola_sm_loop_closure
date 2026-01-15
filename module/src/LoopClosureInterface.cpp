// -----------------------------------------------------------------------------
//   A Modular Optimization framework for Localization and mApping  (MOLA)
//
// Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
// Licensed under the GNU GPL v3.
// -----------------------------------------------------------------------------

#include <mola_sm_loop_closure/LoopClosureInterface.h>

using namespace mola;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(LoopClosureInterface, mrpt::rtti::CObject, mola);

LoopClosureInterface::LoopClosureInterface()  = default;
LoopClosureInterface::~LoopClosureInterface() = default;
