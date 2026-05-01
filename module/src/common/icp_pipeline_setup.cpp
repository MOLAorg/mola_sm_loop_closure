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

#include <mola_sm_loop_closure/common/icp_pipeline_setup.h>
#include <mola_yaml/yaml_helpers.h>
#include <mp2p_icp/icp_pipeline_from_yaml.h>
#include <mp2p_icp_filters/FilterBase.h>

mp2p_icp::Parameters mola::lc_common::load_icp_pipeline_from_yaml(
    const mrpt::containers::yaml& cfg, PerThreadIcpPipeline& out,
    const std::string& /*threshold_sigma_initial*/, const std::string& /*threshold_sigma_final*/)
{
    ENSURE_YAML_ENTRY_EXISTS(cfg, "icp_settings");

    const auto [icp, icpParams] = mp2p_icp::icp_pipeline_from_yaml(cfg["icp_settings"]);
    out.icp                     = icp;
    out.icp->attachToParameterSource(out.parameter_source);

    if (cfg.has("observations_generator") && !cfg["observations_generator"].isNullNode())
    {
        out.obs_generators = mp2p_icp_filters::generators_from_yaml(cfg["observations_generator"]);
    }
    else
    {
        auto defaultGen = mp2p_icp_filters::Generator::Create();
        defaultGen->initialize({});
        out.obs_generators.push_back(defaultGen);
    }
    mp2p_icp::AttachToParameterSource(out.obs_generators, out.parameter_source);

    if (cfg.has("observations_filter") && !cfg["observations_filter"].isNullNode())
    {
        out.pc_filter = mp2p_icp_filters::filter_pipeline_from_yaml(cfg["observations_filter"]);
        mp2p_icp::AttachToParameterSource(out.pc_filter, out.parameter_source);
    }

    // expr_threshold_sigma_* are compiled lazily at runtime (not here) because
    // they require runtime symbol tables not available during initialization.

    return icpParams;
}
