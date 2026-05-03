[![CI ROS](https://github.com/MOLAorg/mola_sm_loop_closure/actions/workflows/ros-build.yml/badge.svg)](https://github.com/MOLAorg/mola_sm_loop_closure/actions/workflows/ros-build.yml)
[![CI Check clang-format](https://github.com/MOLAorg/mola_sm_loop_closure/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MOLAorg/mola_sm_loop_closure/actions/workflows/check-clang-format.yml)
[![Docs](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://docs.mola-slam.org/mola_sm_loop_closure/)
[![codecov](https://codecov.io/gh/MOLAorg/mola_sm_loop_closure/graph/badge.svg?token=K0DYJAI45R)](https://codecov.io/gh/MOLAorg/mola_sm_loop_closure)


| Distro | Build dev | Build releases | Stable version |
| ---    | ---       | ---            | ---         |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_sm_loop_closure__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_sm_loop_closure__ubuntu_jammy_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_sm_loop_closure__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_sm_loop_closure__ubuntu_jammy_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_sm_loop_closure__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_sm_loop_closure__ubuntu_jammy_arm64__binary/) | [![Version](https://img.shields.io/ros/v/humble/mola_sm_loop_closure)](https://index.ros.org/?search_packages=true&pkgs=mola_sm_loop_closure) |
| ROS 2 Jazzy @ u24.04 | [![Build Status](https://build.ros2.org/job/Jdev__mola_sm_loop_closure__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_sm_loop_closure__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Jbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_sm_loop_closure)](https://index.ros.org/?search_packages=true&pkgs=mola_sm_loop_closure) | 
| ROS 2 Kilted @ u24.04 | [![Build Status](https://build.ros2.org/job/Kdev__mola_sm_loop_closure__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mola_sm_loop_closure__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Kbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Kbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/kilted/mola_sm_loop_closure)](https://index.ros.org/?search_packages=true&pkgs=mola_sm_loop_closure) | 
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_sm_loop_closure__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_sm_loop_closure__ubuntu_noble_amd64/) | amd64 [![Build Status](https://build.ros2.org/job/Rbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mola_sm_loop_closure__ubuntu_noble_amd64__binary/) <br> arm64 [![Build Status](https://build.ros2.org/job/Rbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uNv8__mola_sm_loop_closure__ubuntu_noble_arm64__binary/) | [![Version](https://img.shields.io/ros/v/rolling/mola_sm_loop_closure)](https://index.ros.org/?search_packages=true&pkgs=mola_sm_loop_closure) |



# mola_sm_loop_closure

Offline loop-closure engine for MOLA `CSimpleMap` files.  Given an input
simplemap produced by any MOLA odometry front-end, the package re-optimises
all keyframe poses by detecting and closing loops, then writes the corrected
simplemap back to disk.

## Two algorithms

| Algorithm | Class | Best for |
|-----------|-------|----------|
| **SimplemapLoopClosure** | `mola::SimplemapLoopClosure` | Long maps with noticeable drift; groups keyframes into submaps and runs heavy point-cloud ICP between submap pairs |
| **FrameToFrameLoopClosure** | `mola::FrameToFrameLoopClosure` | GNSS-augmented datasets or quick re-optimisation; runs frame-to-frame ICP with a GNC graph optimizer |

## CLI usage

```bash
# SimplemapLoopClosure (default algorithm):
mola-sm-lc-cli -i in.simplemap -o out.simplemap \
    -p pipelines/loop-closure-lidar3d-gicp.yaml

# FrameToFrameLoopClosure:
mola-sm-lc-cli -i in.simplemap -o out.simplemap \
    -a mola::FrameToFrameLoopClosure \
    -p pipelines/loop-closure-f2f-lidar3d-gicp.yaml
```

## Available pipelines

| YAML file | Sensor type | Algorithm |
|-----------|-------------|-----------|
| `loop-closure-lidar3d-gicp.yaml` | 3-D LiDAR (GICP) | SimplemapLoopClosure |
| `loop-closure-lidar3d-icp.yaml`  | 3-D LiDAR (point-to-point ICP) | SimplemapLoopClosure |
| `loop-closure-lidar2d.yaml`      | 2-D LiDAR | SimplemapLoopClosure |
| `loop-closure-f2f-lidar3d-gicp.yaml` | 3-D LiDAR (GICP) | FrameToFrameLoopClosure |

## Key YAML knobs

**SimplemapLoopClosure**
- `submap_max_absolute_length` / `submap_min_absolute_length` : controls submap granularity.
- `assume_planar_world: true` enables annealed soft planar constraints (z, roll, pitch).
  - `planar_world_initial_sigma_z`, `planar_world_initial_sigma_ang`, `planar_world_annealing_rounds` : tune the annealing schedule.
  - `planar_world_hard_flatten: true` restores the old hard-flattening behaviour.
- `use_gnss: true` / `gnss_add_horizontality: true` : GNSS-assisted global alignment.
  - `gnss_factor_strategy: "submap"` (default, scalable) or `"per_kf"` (sensor-pose-aware, larger graph).
  - `gnss_max_uncertainty_horiz` (default `20.0` m) : reject GPS readings whose horizontal ENU uncertainty (√(σ_E²+σ_N²)) exceeds this threshold.
  - `gnss_max_uncertainty_vert` (default `40.0` m) : reject GPS readings whose vertical ENU uncertainty (σ_U) exceeds this threshold.
  - Stats on accepted/rejected GNSS readings are printed at the INFO log level after processing.
- `use_imu_gravity: true` / `imu_gravity_sigma_deg` : IMU-derived gravity-alignment factors added in stage 1.

**FrameToFrameLoopClosure**
- `lc_candidate_strategy` : `DISTANCE_STRATIFIED` (default), `PROXIMITY_ONLY`, or `MULTI_OBJECTIVE`.
- `assume_planar_world: true` : planar-world annealing (subset of SM options; IMU-gravity options are not exposed by `FrameToFrameLoopClosure::Parameters`).
- `use_gnss: true` : per-keyframe GNSS factors (`FactorGnssEnu`).
  - `gnss_max_uncertainty_horiz` (default `20.0` m) : reject GPS readings whose horizontal ENU uncertainty exceeds this threshold.
  - `gnss_max_uncertainty_vert` (default `40.0` m) : reject GPS readings whose vertical ENU uncertainty exceeds this threshold.
- `manual_loop_constraints` : list of hand-specified loop closure edges (by UNIX timestamp pair). Each entry supports:
  - `timestamp_i`, `timestamp_j` : UNIX timestamps identifying the two keyframes.
  - `sigma_xyz` : positional uncertainty [m].
  - `trust_as_inlier` (default `false`): if `true`, the constraint is added to the GNC set of known inliers and will not be downweighted by the robust optimizer.
- `use_kiss_matcher: true` : use [KISS-Matcher](https://github.com/mit-spark/kiss-matcher) global registration to seed the ICP initial guess for each loop-closure candidate (default: `false`; requires the `third_party/kiss-matcher` submodule to be populated).
  - `kiss_matcher_resolution` (default `1.0` m) : voxel size for KISS-Matcher feature extraction; controls `normal_radius ≈ 3×` and `fpfh_radius ≈ 5×`.
  - `kiss_matcher_layer` (default `"points_to_register_points"`) : name of the `mp2p_icp::metric_map_t` layer whose points are fed into KISS-Matcher.

See the [online tutorial](https://docs.mola-slam.org/latest/tutorial-ouster-mapping-lc.html) for a step-by-step example.

# License
Copyright (C) 2018-2026 Jose Luis Blanco <jlblanco@ual.es>, University of Almeria

This package is released under the GNU GPL v3 license as open source, with the main 
intention of being useful for research and evaluation purposes.
Commercial licenses [available upon request](https://docs.mola-slam.org/latest/solutions.html).

Contributions require acceptance of the Contributor License Agreement (CLA).
