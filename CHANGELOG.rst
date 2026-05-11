^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_sm_loop_closure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2026-05-11)
------------------
* f2f pipeline: switch to multi-objective as default strategy
* Merge pull request `#15 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/15>`_ from MOLAorg/simplify-ci
  CI: simplify clang-format helpers and use ros: docker images for stable builds
* fix: wrap find -iname predicates in parentheses to scope them to listed dirs
  Without grouping, the -o operators apply at top level and -print0 only
  attaches to the last alternative, so files outside the intended directories
  can be matched.
* CI: simplify clang-format helpers and use ros: docker images for stable builds
  - Replace complex Python-based clang_git_format with a simple scripts/formatter.sh supporting --check
  - Standardize formatter to scripts/formatter.sh (moved from root formatter.sh)
  - Simplify check-clang-format.yml to just apt-install clang-format-14 and run the script
  - Use ros:humble / ros:jazzy pre-built images for stable CI builds (faster, no setup-ros needed)
* fix:as libflann as build_depend as needed for kiss-matcher
* CI: sensible job names
* bump min req cmake version to 3.22
* feat: live gui, show GNSS data points
* update package.xml build deps
* Merge pull request `#14 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/14>`_ from MOLAorg/improve-lc-hypothesis-coverage
  Improve lc hypothesis coverage
* fix: deduplicate candidates by block-pair ID before selection in find_loop_candidates
  Multiple (i,j) frame pairs can round to the same (frameGroup_i, frameGroup_j)
  block and all pass the alreadyChecked filter (which only holds pairs from
  previous rounds). All of them survived into the scored candidate list, so
  max_lc_candidates were returned but the evaluation loop in process() silently
  skipped all but the first occurrence of each block pair -- causing far fewer
  ICP evaluations than expected.
  Fix: after scoring (step 1) and before selection (step 2), deduplicate each
  candidate container by block-pair ID, keeping only the highest-scored entry
  per block. This is applied to both the flat candidates vector (PROXIMITY_ONLY,
  MULTI_OBJECTIVE) and each distance bin (DISTANCE_STRATIFIED), so
  max_lc_candidates now matches the number of candidates that will actually
  be evaluated.
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
* fix: MULTI_OBJECTIVE coverage term now uses spatial midpoints instead of sin formula
  The old coverage score used abs(sin(pi * topologicalMidpoint / totalFrames)),
  which only rewards candidates near the center of the frame index range and
  ignores actual map geometry. This caused clustering in the same physical region
  on non-linear trajectories (U-shapes, figure-8s, long straights).
  New approach:
  - Add LoopCandidate::spatialMidpoint ((pose_i + pose_j) / 2, computed once
  per candidate in find_loop_candidates()).
  - Track selectedMidpoints alongside selectedDistances in the greedy selection
  loop.
  - Coverage score penalizes candidates whose spatial midpoint is within ~20 m
  of an already-selected midpoint, forcing the selected set to cover different
  map areas regardless of trajectory shape.
  - Refactor score_multi_objective() to accept a MultiObjectiveArgs struct
  instead of a long flat parameter list, making call sites self-documenting.
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
* fix: MULTI_OBJECTIVE diversity scoring is now actually applied during selection
  selectedDistances was declared but never populated, so score_multi_objective()
  always computed diversity against an empty set (score = 1.0 always). The fix
  replaces the simple sort+truncate step 2 path for MULTI_OBJECTIVE with a greedy
  selection loop: pick the best candidate, record its distance, re-score the rest
  with the updated selectedDistances, repeat. This makes the diversity term a
  real participant in selection instead of a no-op.
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
* fix: live preview GNC stats no longer reset to zero at each LC round start
  liveStats was zero-initialized at the top of every lcRound loop, causing
  the 3Dscene caption to show "0 inliers, 0 outliers" until the first GNC
  optimization ran in that round. Carry lastGncInliers / lastGncOutliers
  forward so the caption always reflects the most recent optimizer result.
  Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
* feat: show better inliers/outliers stats
* feat: manual LC now has an optional bypass for inliers
* Merge pull request `#13 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/13>`_ from MOLAorg/feat/live-gui-3dscene-file
  feat: show 'live' progress via 3Dscene file updates
* feat: show 'live' progress via 3Dscene file updates
* add new mp2p_icp flags: force_final_pairings_for_quality
* Add Codecov badge to README
* Merge pull request `#12 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/12>`_ from MOLAorg/feat/f2f-integrate-kiss-matcher
  feat: Incorporate KISS-Matcher as optional seeder for initial ICP poses
* fix: CI code coverage
* chore: add more INFO traces
* Fixes: cmake gating, don't expose kiss-matcher headers, manual LC should be inlier
* feat: Incorporate KISS-Matcher as optional seeder for initial ICP poses
* FIX: crashes due to missing noise model and different gtsam symbols
* feat: add gnss filter by max uncertainty
* Merge pull request `#11 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/11>`_ from MOLAorg/refactor-common-code
  refactor: unify common code between the two LC algorithms
* CI: Fix for new ROS2 rolling, add codecov upload
* refactor: unify common code between the two LC algorithms
* Contributors: Jose Luis Blanco-Claraco

1.1.0 (2026-04-29)
------------------
* Merge pull request `#10 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/10>`_ from MOLAorg/refactor/lc-common-helpers
  refactor: extract lc_common helpers; port planar-world annealing + GNC to SM
* refactor: replace two-pass LM with single GNC pass in SimplemapLoopClosure
* refactor: extract lc_common helpers; port planar-world annealing + GNC to SM
* Merge pull request `#9 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/9>`_ from MOLAorg/feat/planar-world-in-f2f
  feat: optional f2f planar world soft-constraints
* feat: optional f2f planar world soft-constraints
* Merge pull request `#8 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/8>`_ from MOLAorg/feat/manual-lc
  Support for optional manual hints for LC
* Support for optional manual hints for LC
* Merge pull request `#7 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/7>`_ from MOLAorg/feat/more-flexible-icp-sigmas-and-logging
  Expose more env vars
* Expose more env vars
* f2f pipeline file: add env var MOLA_DESKEW_IGNORE_ACCELEROMETER
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2026-04-14)
------------------
* package.xml: add missing test-dep
* add unit tests
* cli: use output directory as default for debug output files, not input
* Install pipelines so they are accessible under 'share'
* Make the package discoverage by ament
* Merge pull request `#6 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/6>`_ from MOLAorg/save-ram
  F2F algorithm: save ram in offline runs
* More memory efficient loop
* Bump minimum cmake version to 3.7
* Reuse mp2p_icp::update_velocity_buffer_from_obs() from mp2p_icp
  Removes duplicated code in this repo now that mp2p_icp>=2.5.0 is available in all distributions
* Fix build against mp2p_icp <2.6.0
* icp debug log files: save only good edges
* F2F algorithm: save ram in offline runs
* sm2mm pipeline yaml: expose more params via env vars
* Expose more parameters for F2F algorithm
* Debug feature: enable saving 3Dscene files per optimization round
* Use Graduated Non-Convexity (GNC) optimizer for superior outlier rejection
* FIX: Reverted logic in formula for adaptive threshold
* Add formal CLA
* Merge pull request `#5 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/5>`_ from MOLAorg/feat/smart-ram-lazy-unload
  Lazy unload keyframe clouds to keep RAM usage bounded
* Contributors: Jose Luis Blanco-Claraco

0.2.0 (2026-03-03)
------------------
* Fix build against different gtsam versions
* Add optional generation of .3Dscene files with loop-closure visualizations
* Tolerate missing external files without throwing
* Better logging and expose more env var params
* Add GNSS uncertainty multiplier
* Use debug files prefix
* Add more frame-to-frame LC selection algorithms
* Merge pull request `#4 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/4>`_ from MOLAorg/feat/f2f-fixes
  Add more parameters to f2f method
* Add new params for f2f method
* pipeline file: expose more params and fix reversed sigma logic
* Fix: prior should be weak for GNSS to transform to ENU frame
* fix copyright headers
* Merge pull request `#3 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/3>`_ from MOLAorg/feat/new-f2f-algo
  Add alternative frame-to-frame LC algorithm
* Provide virtual LoopClosureInterface to select the algorithm from a CLI argument
* Add alternative frame-to-frame LC algorithm
* Merge pull request `#2 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/2>`_ from MOLAorg/feat/add-ci-and-badges
  Add CI workflows and clang-format linting infrastructure
* package.xml: add FILE tag
* Merge pull request `#1 <https://github.com/MOLAorg/mola_sm_loop_closure/issues/1>`_ from MOLAorg/feature/refactor-pkg-mola-gtsam-factors
  Progress moving code to new package mola_gtsam_factors
* Update dependencies after refactorization into mola_georeferencing
* Progress moving code to new package mola_gtsam_factors
* Protect against invalid GPS cov entries
* Add param gnss_minimum_uncertainty_xyz; dump trajectories as TUM files.
* simplify: remove obsolete parameters not used anymore here
* Remove unused parameter (GPS observations are automatically detected anyway)
* Fix: ensure minimum uncertainty in edges
* Fix deskewing generating sm2mm for local maps
* Implement missing loading of new LocalVelocityBuffer while doing submaps "sm2mm"
* Don't use absolute paths for debug icplog files
* FIX: Detect the error case of no GNSS observations in sm-georeference-cli
* Add gicp pipeline
* bump minimum cmake version to 3.5
* add .clang-tidy rules and better vscode integration
* Expose more env vars in the pipeline
* robust kernels; optional horizontality
* progress with submaps
* implement submaps
* use new mrpt enu covariance field
* done georeferencing cli
* progress
* progress cli app interface
* initial structure
* Initial commit
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco
