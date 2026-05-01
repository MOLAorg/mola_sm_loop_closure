/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/
#pragma once

#include <mp2p_icp/ICP.h>
#include <mp2p_icp/Parameterizable.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>

namespace mola::lc_common
{
/** Per-thread ICP pipeline state shared by both SimplemapLoopClosure and
 *  FrameToFrameLoopClosure.  SM-specific pipelines (obs2map_merge,
 *  local_map_generators, submap_final_filter) are kept in their owner class.
 */
struct PerThreadIcpPipeline
{
    mp2p_icp::ParameterSource              parameter_source;
    mp2p_icp::ICP::Ptr                     icp;
    mp2p_icp_filters::GeneratorSet         obs_generators;
    mp2p_icp_filters::FilterPipeline       pc_filter;
    mrpt::expr::CRuntimeCompiledExpression expr_threshold_sigma_initial;
    mrpt::expr::CRuntimeCompiledExpression expr_threshold_sigma_final;
};

/** Load the shared ICP pipeline from YAML configuration.
 *
 * Reads \c cfg["icp_settings"], \c cfg["observations_generator"] and
 * \c cfg["observations_filter"] and populates \a out accordingly.  The
 * threshold-sigma expression fields are left un-compiled (they require runtime
 * variable tables and are compiled lazily on first use).
 *
 * @return the mp2p_icp::Parameters extracted from the ICP settings.
 */
mp2p_icp::Parameters load_icp_pipeline_from_yaml(
    const mrpt::containers::yaml& cfg, PerThreadIcpPipeline& out,
    const std::string& threshold_sigma_initial, const std::string& threshold_sigma_final);

}  // namespace mola::lc_common
