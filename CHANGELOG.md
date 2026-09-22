# Change Log

## 2026-07-16

### Move all upstairs action gates 1cm forward

- Moved the upstairs planned target from `0.520m` to `0.510m`, live-Tag/final-visual completion from `0.570m` to `0.560m`, and no-Tag visual fallback/detector trigger from `0.620m` to `0.610m`. The `1.70m` AprilTag branch-entry distance is unchanged.

### Close the field-tested pole exit and correct the robot footprint

- Compared three physical-test logs. All four poles stayed locked and cross-track control remained active; every run was stopped only by the fixed `35s` route watchdog. One run still had `1.36m` remaining and therefore recorded only `2/3` mandatory zones, while another entered `3/3` at about `34.98s` and was incorrectly marked `FAILED` roughly `0.11s` later.
- Increased only the continuous path watchdog to `50s`. After all three mandatory zones and the exit tail point are reached, the controller now enters an independent mode-6 final-turn phase. It derives the row direction from the run-local P3-to-P2 fit, closes a `+90deg` counter-clockwise yaw with coarse/fine commands, requires a `3deg`, `6deg/s`, `0.20s` stable gate, and publishes `FINISHED` only after that turn. The turn has its own `12s` watchdog.
- Corrected the robot dimensions from `0.50 x 0.40m` to `0.66 x 0.46m`. Safety now computes the distance between each pole surface and the current oriented rectangular footprint instead of comparing only center distance. The configured runtime body margin is `4cm`; the generated route must retain at least `8cm` planned footprint margin.

### Add run-local LiDAR control for the mirrored right-angle pole route

- Added a continuous slalom controller that locks P4 from a seven-frame raw `/livox/lidar` window, creates a transient obstacle frame from the current body heading, and progressively refits P3, P2, and P1 through 20cm sequential topology gates. No configured `camera_init` position is used.
- Replaced the stop-turn 14-point execution with a Catmull-Rom path and differential lookahead tracking. The path explicitly passes the three rule-book circular mandatory zones and keeps a planned dog-center clearance above 35cm; runtime clearance below 32cm, stale raw cloud, stale odometry, or 28cm cross-track error stops the robot.
- Enabled the same LiDAR parameters in V1 and V4. Before any LiDAR motion, a failed P4 lock can fall back to the unchanged AprilTag pre-align and 14-point route. After motion starts, failures stop and report `FAILED` instead of changing coordinate bases mid-route. Set `lidar_slalom_enabled: false` for immediate legacy rollback.
- Offline replay of both recorded bags detected P4 in every evaluated seven-frame window, reached a two-sample lock in about 0.35s, and sequentially recovered all four poles. Added geometry, mandatory-zone, clearance, steering-sign, and configuration regression tests.

### Synchronize V1 pole control support with the validated V4 configuration

- Synchronized the V1 AprilTag trigger, fixed-route-yaw pre-alignment, lateral correction, precision turn braking, and disabled retry-bypass settings with V4.
- Preserved V1's independently tuned turn deltas, forward distances, arrival tolerances, and per-segment tracking gains instead of replacing its route with the V4 geometry.
- Added a configuration regression test that requires all non-route V1 parameters to stay equal to V4 while confirming that the two route geometries remain distinct.

## 2026-07-15

### Match standard-route sandpit turns to the V4 pole pre-alignment

- Changed the standard-route sandpit polyline turn-out/turn-back commands to the V4 pole pre-alignment's mode-6 differential pairs: `-0.10/+0.25` counter-clockwise and `+0.25/-0.10` clockwise. The existing remaining-angle scale continues to slow both sides near the target.
- Limited the locked ID10 lateral plan to `65%` of the entry-time stable lateral error. The existing plan lock remains authoritative through the turn, expected Tag loss, shift, and turn-back; later Tag samples do not rewrite the active shift distance.
- Added a focused configuration regression check for the ID10 frame mapping, the `0.65` distance scale, and exact parity with the V4 pole pre-alignment turn mode and four base turn norms.
- Rollback: restore the four asymmetric yaw turn norms and set `polyline_distance_scale: 1.00` in `obstacle_waypoints.yaml`.

### Lock limit-bar yaw globally and widen only its guarded S-curve envelope

- Limit-bar trigger geometry is now locked from exactly three recent Tag frames by selecting the frame with the smallest absolute centerline error and keeping its paired depth. The locked point is transformed into the final global-yaw frame with Odometry yaw only; completion-stage single-frame Tag jitter can no longer enlarge the lateral route.
- Limit-bar fixed-yaw pre-alignment now publishes zero wheel commands during its hold interval, so the robot does not keep turning while the yaw gate is already satisfied.
- Added `limit_bar_tag_yaw_reference_mode` with `fixed_absolute` selected for the field configuration. The limit-bar now aligns to runtime `route_zero + 180deg`, or absolute `180deg` when no runtime route reference is available. Tag ID12 remains authoritative for trigger, forward distance, and lateral centerline geometry; `tag_lateral` restores the previous Tag-derived yaw.
- Changed the limit-bar pre-alignment from visual `pose_pitch` feedback to odometry-backed `fixed_absolute_yaw`, removing the observed alternating Tag-pose corrections while retaining the same configurable timeout and hold gates.
- Added `limit_bar_lateral_s_curve_max_heading_deg: 36.0`. Only the limit-bar uses this relaxed envelope, allowing the observed `0.867m/+0.314m` route with about `34.2deg` peak heading; high-wall and upstairs planning remain on the shared `30deg` limit, and the existing `2.5/m` curvature, cross-track, endpoint, and overshoot guards remain unchanged.
- Lateral phase completion is now evaluated before its watchdog. A turn already inside the yaw gate receives a limit-bar-only `0.60s` yaw-rate settling grace at the watchdog boundary, then transitions normally once the existing yaw-rate and hold gates pass; genuinely incomplete phases still stop on the configured watchdog.
- Rollback: set `limit_bar_tag_yaw_reference_mode: tag_lateral`, restore `limit_bar_lateral_pre_align_mode: pose_pitch` plus visual feedback, set `limit_bar_lateral_s_curve_max_heading_deg: 30.0`, or disable `limit_bar_lateral_s_curve_control_enabled` to restore the old 90deg path.

### Keep limit-bar pre-duck yaw on the Tag-aligned route

- Changed only the limit-bar pre-duck yaw source and turn command. Both actual S-curve and legacy-fallback completion now default to the locked `base_yaw` already used by Tag pose pre-alignment, lateral return, and final forward travel, instead of switching from the observed `160.9deg` route to absolute `camera_init 180deg`.
- Added `limit_bar_pre_duck_yaw_reference_mode`. `base_yaw` is the field default; `route_zero_plus_offset` restores the previous runtime route-zero plus configured `180deg` behavior, including absolute `180deg` fallback when no runtime reference exists.
- Added a symmetric mode-6 stationary turn for this phase, defaulting to `-0.08/+0.08` and shared by S-curve and legacy fallback. Set `limit_bar_pre_duck_stationary_turn_enabled: false` to restore the previous asymmetric overlay command.
- Route feasibility limits remain unchanged: the observed `0.782m/+0.293m` candidate still safely falls back because its `35.1deg` peak heading exceeds the existing `30deg` limit.

### Disable the incorrect post-stairs absolute-zero alignment

- Disabled only `post_stairs_slope_entry_initial_yaw_align_enabled`. With the existing `0.00m` fixed-entry distance and `post_stairs_slope_search_after_entry: true`, stairs completion now reaches SEARCH on the next supervisor cycle without issuing a yaw-turn command.
- Reason: the field run finished stairs near `camera_init +180deg` with the limit bar already ahead, but the independent forced entry alignment bypassed `slope_align_enabled: false` and turned toward absolute `0deg`. Limit-bar, high-wall, upstairs, detected-slope, and retry-specific yaw alignment remain unchanged.
- Rollback: set `post_stairs_slope_entry_initial_yaw_align_enabled: true`.

## 2026-07-14

### Enable guarded short-distance limit-bar S-curve motion

- Added a dedicated limit-bar actual S-curve controller instead of routing the `limit_bar_prepare` action through the high-wall controller. Enabled only the focused limit-bar shadow/control switches; setting `limit_bar_lateral_s_curve_control_enabled: false` restores its validated legacy `90deg-shift-90deg` path, and infeasible plans still fall back automatically before motion.
- The field camera-to-Tag distance is only about `1.0-1.1m`. The existing `0.50m` pre-duck target clearance remains unchanged, leaving roughly `0.50-0.60m` for the curve. The first test uses `0.32/0.32` cruise, `0.20/0.20` over the final `0.18m`, an `8s` watchdog, a `3cm` forward endpoint tolerance, a `5cm` lateral endpoint tolerance, and an `8cm` forward-overshoot hard stop.
- Route feasibility still uses the validated `30deg` heading and `2.5/m` curvature limits. For example, a `0.55m` route accepts about `0.12m` lateral correction; larger short-distance corrections use the legacy path rather than reducing the `0.50m` clearance before crouching.
- Path-yaw recovery, endpoint alignment, and the following global pre-duck yaw alignment use symmetric mode-6 wheel commands with zero average demand. The controller stops for `0.20s`, requires the existing `2deg`/yaw-rate gate and `0.30s` hold, then performs the existing global yaw alignment before entering the crouch sequence.

### Keep upstairs S-curve final alignment stationary and latched

- Changed only the upstairs actual S-curve controller, its focused parameters, and regression tests. Other upstairs paths and all high-wall, limit-bar, and pole motion remain unchanged.
- When odometry reaches the route endpoint before the action distance, yaw errors above `4deg` are corrected against the locked base yaw before the existing `0.15` confirmation creep. The correction and the final `1deg` alignment now use a symmetric mode-6 `-0.08/+0.08` command with zero average wheel demand, instead of inheriting the asymmetric overlay that moved the robot forward while turning.
- Once Tag/visual action distance is confirmed, completion stays authoritative through settle and final yaw alignment. Route-end forward overshoot can no longer abort that already-confirmed state; cross-track protection, final lateral recheck, the `1deg` yaw gate, and the `12s` phase watchdog remain active.
- Rollback: set `upstairs_lateral_s_curve_control_enabled: false` to restore the legacy upstairs motion. Setting `upstairs_lateral_s_curve_control_precreep_yaw_gate_deg: 15.0` restores the previous later pre-creep yaw threshold while retaining stationary final turns.

### Enable the first guarded upstairs S-curve motion test

- Added an upstairs-specific actual S-curve controller and enabled only its focused shadow/control switches after high-wall testing completed. Limit-bar remains on the legacy `90deg-shift-90deg` path, and setting `upstairs_lateral_s_curve_control_enabled: false` immediately restores that path for upstairs.
- Upstairs reuses the validated route-frame lookahead and cross-track controller but starts conservatively at `0.40/0.40`, slows to `0.25/0.25` for the final `0.25m`, and retains its existing `1deg` final yaw gate before `step_once`. Its independent watchdog is `12s`, cross-track protection remains `0.20m`, and forward overshoot is limited to `0.20m`.
- The existing upstairs action-distance semantics are preserved: fresh Tag distance has priority, visual detection remains the fallback, and odometry-first arrival creeps at `0.15/0.15` for at most `0.18m` while waiting for confirmation. A distance candidate cannot finish more than `0.25m` before the route endpoint, and once confirmed it stays latched through stop/yaw alignment even if the next perception frame drops.
- Added regression coverage for branch-specific tracking, early distance rejection, guarded confirmation creep, latched completion, branch configuration, and the upstairs-specific timeout.

### Modestly raise high-wall S-curve cruise speed and tighten pre-jump yaw

- Changed only the focused high-wall S-curve configuration and regression assertions; the slow segment, route tracking gains, safety gates, and other obstacle flows remain unchanged.
- Raised cruise norms from `0.40/0.40` to `0.43/0.43` after the offset stress run held cross-track error to about `3.6cm` and peak runtime yaw error to `12.7deg`. The final `0.25m` still uses `0.25/0.25`.
- Tightened normal completion yaw tolerance from `6deg` to `3deg`, and retreat-recovery completion tolerance from `10deg` to `4deg`. The yaw-rate gate, `0.30s` settle/hold, independent fine turn, and forward recovery remain active.
- Rollback: restore cruise to `0.40/0.40` and yaw tolerances to `6deg`/`10deg`.

### Reject single-frame high-wall visual stop-line outliers

- Changed only the high-wall S-curve visual completion gate, focused configuration, and regression tests. The detector output and legacy high-wall, limit-bar, upstairs, and pole motion paths remain unchanged.
- A visual stop-line candidate now needs either three close samples or `0.15s` of continuous close detection, no more than `0.35m` of planned route remaining, and no more than `0.40m` disagreement with a fresh AprilTag distance. When no fresh Tag exists, the other gates remain sufficient.
- Rejected candidates are logged and ignored while S-curve control continues. This prevents a one-frame `0.192m` depth outlier at only `0.012/1.073m` route progress from entering `FAILED`; confirmed near-end visual completion keeps the existing lateral safety check and completion flow.
- Rollback: set the four `hurdle_lateral_s_curve_visual_jump_*` confirmation/progress/Tag guard parameters to `1`, `0`, `0`, and `0` respectively.

### Recover high-wall completion retreat instead of ending the flow

- Changed only the high-wall S-curve completion controller, focused configuration, and regression tests. Limit-bar, upstairs, and pole motion remain unchanged.
- When final yaw correction retreats more than `5cm` from the closest post-arrival point, the controller now freezes a recovery target `1.5cm` behind that closest point and walks forward at low all-positive norms while applying a bounded yaw correction. Once distance and yaw settle, the existing completion hold continues into jump wait instead of entering `FAILED`.
- Recovery is accepted only while endpoint lateral error is within `0.15m`; the existing lateral-drift, route cross-track, and forward-overshoot safety stops remain active. If yaw still needs correction after `3cm` of extra recovery travel, control returns to the independent final turn and repeats forward compensation if that turn retreats again.
- Rollback: set `lateral_s_curve_control_completion_recovery_enabled: false` to restore the previous stop-and-fail behavior for completion retreat.

### Stabilize high-wall S-curve entry and protect the jump-line pose

- Changed the high-wall supervisor control, focused configuration, and regression tests. Limit-bar and upstairs motion selection remain unchanged.
- High-wall fixed-yaw pre-alignment now stops during its in-tolerance hold instead of continuing to rotate itself back out of tolerance. Its tolerance is `3deg`, and any required correction uses independent low-amplitude turn norms rather than the later-loaded limit-bar overlay values.
- S-curve lookahead is `0.20m` instead of `0.12m`, so the controller requests the initial bend and the return to route-zero earlier while retaining the `18deg` walking error gate and all route feasibility limits.
- At the visual/odometry arrival line the controller now stops for `0.30s` before evaluating yaw, accepts a high-wall-specific `6deg` final yaw window, and uses independent low-amplitude final-align turns. Completion rechecks lateral error; more than `5cm` of retreat from the closest post-arrival point or more than `5cm` of added lateral drift stops the run instead of allowing repeated corrective turns.
- Rollback: disable the two `hurdle_*_independent_turn_enabled` switches and `hurdle_lateral_pre_align_stop_during_hold_enabled`, restore lookahead/yaw tolerance to `0.12m`/`2deg`, and set arrival settle plus both completion drift limits to `0`.

### Raise the guarded high-wall S-curve heading limit to 30 degrees

- Changed only the focused supervisor configuration and its regression assertion. High-wall is still the only branch with actual S-curve control enabled.
- The latest `0.980m/+0.279m` candidate has a `28.1deg` peak heading and `1.57/m` maximum curvature. The configured planning limit is now `30deg`, so this route keeps its full lateral correction instead of falling back; the `18deg` runtime yaw-error gate, `2.5/m` curvature limit, speed, slowdown, endpoint checks, and overshoot guards are unchanged.
- Rollback: restore `lateral_s_curve_shadow_max_heading_deg: 25.0`.

### Keep marginal high-wall routes on the S-curve and guard legacy fallback distance

- Changed the obstacle supervisor, focused configuration, and tests; limit-bar and upstairs branches are unchanged.
- A high-wall S-curve that only marginally exceeds the `25deg` heading limit now keeps that limit and reduces its lateral endpoint by at most 10%. The observed `1.083m/+0.278m` candidate becomes approximately `1.083m/+0.269m`, avoiding a full legacy fallback for a `0.7deg` excess. Larger reductions, curvature violations, and other infeasible plans still fall back before motion.
- When high wall does use the legacy `90deg-shift-90deg` path, the final forward distance is recomputed after turning back from fresh Tag depth and a fresh visual stop-line cap. Relocking may only shorten the original plan and is capped at `0.30m`; the final `0.25m` uses `0.25/0.25` walking norms, with a `0.10m` odometry overshoot hard stop.
- Rollback: set `hurdle_lateral_s_curve_adaptive_clamp_enabled: false`, set `hurdle_lateral_shift_legacy_relock_final_forward_enabled: false`, set `hurdle_lateral_shift_legacy_slowdown_distance_m: 0.0`, and set `hurdle_lateral_shift_legacy_max_forward_overshoot_m: 0.0` independently.

### Enable guarded high-wall S-curve motion and restore the post-stairs yaw align

- Changed the obstacle supervisor, focused configuration, and tests.
- High-wall lateral correction now follows the validated quintic route directly with route-frame odometry, lookahead heading, cross-track feedback, a `0.40` cruise command, and a `0.25` final slowdown. Visual early completion and odometry fallback remain active; endpoint lateral/yaw gates, timeout, cross-track, and forward-overshoot guards stop unsafe runs.
- First-motion tuning: the walk-yaw safety gate is `18deg` instead of `12deg` so the observed `13.7-15.1deg` transient stays in continuous curve tracking, while larger errors still point-align. The visual early-completion distance is `0.55m` instead of `0.65m`, placing the jump closer to the `0.50m` Tag plan while retaining `5cm` for frame/command latency.
- Scope and rollback: only `hurdle_lateral_s_curve_control_enabled` is enabled. Limit-bar and upstairs remain on the legacy `90deg-shift-90deg` path. Set `lateral_s_curve_control_enabled: false` for a complete motion rollback; infeasible plans automatically use the legacy path before motion starts.
- Fixed the post-stairs configuration gap where a `0m` entry distance and `slope_align_enabled: false` skipped a logged `12.8deg` yaw error. The new independent `post_stairs_slope_entry_initial_yaw_align_enabled` switch forces only this entry alignment without re-enabling slope-branch alignment; set it to `false` to restore the previous behavior.

### Apply the retried route-zero yaw to selected downstream alignments

- Changed the obstacle supervisor, V4 pole route, focused configuration, and tests.
- Behavior: when a fresh runtime route-zero reference exists, high-wall entry/pre-align/post-align and upstairs entry/pre-align use `route_zero+0deg`, the V4 pole entry and locked route frame use `route_zero-90deg`, and the limit-bar pre-duck alignment uses `route_zero+180deg`. Bridge-local yaw remains independent and other obstacle behavior is unchanged.
- Fallback: set `downstream_route_yaw_reference_enabled: false` for all supervisor consumers, disable an individual `downstream_route_yaw_*_enabled` switch for one branch, or set V4 `runtime_route_yaw_reference_enabled: false` for the pole route. Missing or expired references automatically retain the original `0deg`, `-90deg`, and `180deg` targets.

## 2026-07-13

### Lock V4 pole entry to global -90deg and stop using Tag pose for heading

- Changed `orange_pole_relative_nav.py`, `orange_pole_body_relative_test_v4.yaml`, and focused pre-align tests.
- Reason: the supplied run showed raw Tag `pose_pitch` moving from `-12.70deg` to `+3.86deg` while the robot turned. The five-sample median prevented a long oscillation and the initial Tag-angle phase still finished in about `1.54s`, but using Tag pose made the entry heading and locked route frame vary between runs.
- Behavior: pre-align now supports `fixed_absolute_yaw`. V4 selects it with `pole_pre_align_fixed_yaw_deg: -90.0`, so Odometry alone controls initial heading, turn-back, final heading verification, recovery, and the locked route-frame yaw. Tag ID14 is retained for trigger, lateral offset, and distance only; its pose angle is ignored. The existing 2deg tolerance, 3.5deg hysteresis, 0.20s hold, and 0.70 pose-turn scale remain active.
- Expected timing: this run would remove most of the roughly `1.54s` Tag-angle acquisition/correction, but the two 90deg lateral-correction turns still consumed about `10.30s` and are unchanged by this patch.
- Rollback: set `pole_pre_align_angle_metric: "pose_pitch"`, remove `pole_pre_align_fixed_yaw_deg`, and let the route anchor use the measured pose yaw again.

### Disable the V4 pole retry bypass and damp AprilTag angle alignment

- Changed `orange_pole_relative_nav.py`, the V4 pole configuration, and focused stability tests. V1 was restored to its previous values.
- Reason: two failed pole attempts must no longer select the five-stage pole bypass. The AprilTag pose-angle adjustment also needs lower command amplitude near the acceptance window so it settles into tolerance instead of oscillating across it.
- Behavior: V4 now sets `pole_bypass.enabled: false`. A new `pole_pre_align_pose_turn_scale` parameter defaults to `1.0` and V4 sets it to `0.70`, reducing only initial/final AprilTag pose-angle turn commands by 30%. The lateral shift turn-out/turn-back controller remains unscaled, and the existing `2deg` acceptance tolerance, median filter, hysteresis, fine-turn band, and hold time are unchanged.
- Rollback: set V4 `pole_bypass.enabled` back to `true`, remove or set `pole_pre_align_pose_turn_scale: 1.0`, and remove the pose-only scaling helper.

### Move all upstairs action gates another 0.5cm earlier

- Changed the upstairs supervisor and detector defaults/configuration.
- Behavior: the planned target is now `0.520m`, live-Tag/final-visual completion is `0.570m`, and no-Tag visual fallback/detector trigger is `0.620m`. The `1.70m` AprilTag branch-entry distance remains unchanged.

### Move all upstairs action gates 1.5cm earlier

- Changed the upstairs supervisor and detector defaults/configuration.
- Behavior: the planned target moves from `0.500m` to `0.515m`; the live-Tag and final visual completion gates move from `0.550m` to `0.565m`; the no-Tag visual fallback and detector trigger move from `0.600m` to `0.615m`. The `1.70m` AprilTag branch-entry distance is unchanged.
- Rollback: restore the four action-distance values to `0.500m`, `0.550m`, `0.600m`, and `0.600m` respectively.

### Move all upstairs action distance gates forward by 3cm

- Changed the upstairs supervisor and detector defaults/configuration.
- Reason: the latest full Tag run latched at `0.580m` and reported a final distance of `0.573m`, while the field position was still too far from the step.
- Behavior: the planned target moves from `0.53m` to `0.50m`; with the unchanged `0.05m` tolerance the live-Tag completion gate moves from `0.58m` to `0.55m`; final visual completion also moves to `0.55m`. The no-Tag visual fallback and detector trigger move from `0.66m` to `0.60m` so fallback cannot retain the older farther action point.
- Rollback: restore target/final visual to `0.53m`/`0.58m` and both close visual trigger distances to `0.66m`.

## 2026-07-11

### Replace the upstairs distance-confirm pause with guarded creep

- Changed `src/auto_nav_pkg/auto_nav_pkg/obstacle_race_supervisor.py` and its YAML/test coverage.
- Reason: the latest run reached the odometry tolerance at `1783761314.845` but visual distance did not reach `0.610m` until `1783761321.147`, causing about `6.3s` of stop/turn waiting. The requested final position also needed to move approximately `3cm` forward.
- Behavior: the target distance is now `0.53m`, Tag/visual completion is `0.58m`, and odometry-complete runs use a guarded `0.15/0.15` confirmation creep instead of stopping. Creep is limited to `0.18m` beyond planned travel and is disabled while yaw exceeds the existing realign gate.
- Rollback: restore target/visual distances to `0.56m`/`0.61m` and set `upstairs_final_confirm_creep_max_extra_m` to `0.0`.

## 2026-07-11

### Move the upstairs stop line earlier and slow the final approach

- Changed `src/auto_nav_pkg/auto_nav_pkg/obstacle_race_supervisor.py` and its YAML/test coverage.
- Reason: the latest run planned `1.100m` of final travel but reached `1.167m`; visual distance was latched at `0.519m` and the latest distance was `0.440m` when `step_once` was sent. The previous run also finished near `0.423m`, showing a repeatable final-approach overrun rather than the already-fixed phase fallback.
- Behavior: the upstairs target distance is now `0.56m`, Tag and visual completion lines are aligned at `0.61m`, and only the final `0.25m` of the upstairs approach uses reduced `0.30/0.30` walking norms. High-wall and limit-bar walking remain on their existing speeds.
- Rollback: restore the target to `0.52m`, visual completion to `0.58m`, and set `upstairs_final_slowdown_distance_m` to `0.0`.

### Latch upstairs final-align once the jump distance is reached

- Changed `src/auto_nav_pkg/auto_nav_pkg/obstacle_race_supervisor.py`.
- Reason: a field log showed `upstairs_final_ready=True` at `1783756733.780971332`, so the supervisor switched to a yaw-align command before the jump. On the next cycle the visual/Tag distance confirmation dropped false, but the phase was still internally `final_forward`, so the supervisor sent `[0,0.500,0.500]` again and moved forward before `step_once`.
- Behavior: once the upstairs final distance gate is reached, the supervisor now latches `upstairs_final_align` as a real phase with the triggering distance reason and progress. That phase never re-enters forward walking; it only holds/turns for yaw correction or sends the existing upstairs `step_once`.
- Rollback: remove the `upstairs_final_align_latched_*` fields and the `phase == 'upstairs_final_align'` branch, and restore the old label-only `upstairs_final_align` behavior.

## 2026-07-10

### Recover upstairs entry when the AprilTag TF burst is dropped

- Changed `src/auto_nav_pkg/auto_nav_pkg/obstacle_race_supervisor.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_supervisor.yaml`.
- Reason: the field log repeatedly showed AprilTag ID18 detections and `apriltag` diagnostics reporting `camera_color_optical_frame->upstairs`, while the supervisor still had `tf_samples=0/1`. At the same time, the close-range upstairs detector reached `trigger=true` at about `0.39-0.46m`, but tag mode intentionally ignored that visual trigger and kept commanding search motion.
- Behavior: the raw `/tf` subscription now uses a reliable depth-100 dynamic-TF queue instead of depth 10. The supervisor also preserves the upstairs detector lateral value and adds a guarded close-range fallback: only after the high-wall branch is complete, ID18 was seen within 2 seconds, no fresh upstairs TF exists, and the visual detector is inside its normal `0.66m` trigger does it bypass tag alignment and send the existing upstairs `step_once`. Missing-TF logs now include the age and children of the latest `/tf` message.
- Rollback: set `upstairs_tag_visual_fallback_enabled: false` and restore the `/tf` subscription depth to 10.

## 2026-05-22 22:53 CST

### Wire RealSense QoS launch args through and clarify aligned-depth wait logs

- Changed `src/realsense-ros/realsense2_camera/launch/rs_launch.py`.
- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_detector.py`.
- Changed `src/auto_nav_pkg/config/orange_pole_detector.yaml`.
- Reason: runtime inspection showed `/camera/aligned_depth_to_color/image_raw` was being published, but the previously added `color_qos/depth_qos/...` arguments from the obstacle supervisor launch were not declared by `rs_launch.py`, so the intended `SENSOR_DATA` publisher QoS never took effect. At the same time, `orange_pole_detector` always printed the same “waiting for aligned depth” line whether the depth stream was merely starting up or had stayed disconnected for multiple seconds.
- Behavior: `rs_launch.py` now exposes `color_qos`, `color_info_qos`, `depth_qos`, and `depth_info_qos` launch arguments so supervisor-level QoS overrides actually reach the RealSense node. `orange_pole_detector` now logs the first aligned-depth frame arrival, and if no depth arrives after `depth_wait_warn_after_sec` it upgrades the message to a warning and includes the configured topic name plus whether `camera_info` is already present.
- Rollback: remove the four QoS launch arguments from `rs_launch.py`, remove `depth_wait_warn_after_sec` and the first-depth logging branch from `orange_pole_detector.py`, and delete the matching YAML parameter.

## 2026-05-22 22:32 CST

### Stabilize mirrored final descent and color key debug logs

- Changed `src/auto_nav_pkg/config/obstacle_waypoints_mirror.yaml`.
- Changed `src/auto_nav_pkg/launch/obstacle_race_visual_supervisor.launch.py`.
- Changed `src/apriltag_ros/cfg/tags_36h11.yaml`.
- Added `src/auto_nav_pkg/auto_nav_pkg/log_style.py`.
- Changed `src/auto_nav_pkg/auto_nav_pkg/obstacle_manager.py`.
- Changed `src/auto_nav_pkg/auto_nav_pkg/nav_executor.py`.
- Changed `src/auto_nav_pkg/auto_nav_pkg/slope_branch_detector.py`.
- Changed `src/auto_nav_pkg/auto_nav_pkg/limit_bar_duck_test.py`.
- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_hurdle_jump_test.py`.
- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_detector.py`.
- Changed `src/quadruped_step_mapper/src/cmd_vel_to_serial_node.cpp`.
- Reason: the latest mirrored run no longer failed on `bridge_entry`, `bridge_exit`, or the post-bridge `45deg` spin. The actual field failure moved to `final_descent_approach`: once the downhill segment drifted laterally, the goal moved behind the body, `pose_controller` escalated into in-place turning, and waypoint 9 never completed. The same log set also showed poor AprilTag effective sample continuity and highly uniform terminal colors that made it hard to spot state transitions versus high-rate serial debug lines.
- Behavior: mirrored `final_descent_approach` now uses a fixed forward direct-step override with explicit path-reference start/end publication, `path_correction_profile=2`, and a wider pass-through lateral tolerance so the robot can keep the downhill line and still hand off cleanly to `final_heading_align` after crossing the goal corridor. The main RealSense supervisor launch now forces `color/depth image` and `camera_info` QoS to `SENSOR_DATA`, and AprilTag uses `detector.decimate=1.5` to trade a little image detail for a steadier callback rate. A shared ANSI log-style helper now colors key messages across `obstacle_manager`, `nav_executor`, the HSV detector nodes, and `cmd_vel_to_serial`; the serial bridge also colors `serial tx`, `step_debug`, and state/profile updates by importance.
- Rollback: restore mirrored `final_descent_approach` to the old `/goal_pose` tracking fields, remove the four RealSense QoS launch arguments, set `detector.decimate` back to `1.0`, remove `log_style.py` imports/usages from the Python nodes, and remove the ANSI wrappers from `cmd_vel_to_serial_node.cpp`.

## 2026-05-22 21:20 CST

### Mirrored turn_180_spin stable yaw reference

- Changed `src/auto_nav_pkg/config/obstacle_waypoints_mirror.yaml`.
- Reason: field logs showed the route had already advanced to `waypoint=5/10 turn_180_spin` and was sending `mode=6` spin override, but `yaw_err` stayed around `135deg` while `dist` was already near tolerance. That points to the spin goal yaw being computed from the wrong reference at send time, not a failure to transition out of the crouch segment.
- Behavior: `turn_180_spin` now uses `body_yaw_mode: previous_target_delta`, so its target yaw is derived from the stable `wall_jump_prep` target yaw plus `-85deg`, instead of `current_pose_yaw + (-85deg)`.
- Rollback: remove `body_yaw_mode` from mirrored `turn_180_spin` to restore the default `delta` behavior.

## 2026-05-22 20:47 CST

### Uphill early correction on mirrored ramp

- Changed `src/quadruped_step_mapper/config/cmd_vel_to_serial.yaml`.
- Reason: field logs showed the mirrored uphill segment was armed (`path_profile=3`, `uphill_step_floor=true`) but serial-side recovery still did not engage while `path_lat_err` stayed around `0.008-0.033m` and `path_yaw_err` stayed around `1-2deg`; the robot could drift to the ramp edge before either correction path triggered.
- Behavior: only uphill-specific correction is tightened. `uphill_path_lateral_correction_deadband_m` is reduced from `0.05` to `0.03`, and `uphill_path_yaw_correction_enabled` is turned on so the ramp segment can recover heading earlier without changing ordinary flat-ground behavior.
- Rollback: restore `uphill_path_lateral_correction_deadband_m: 0.05` and `uphill_path_yaw_correction_enabled: false`.

## 2026-05-22 02:18 CST

### RealSense stream-rate restore for AprilTag continuity test

- Changed `src/auto_nav_pkg/launch/obstacle_race_visual_supervisor.launch.py`.
- Reason: with `15fps`, `wall_jump_prep` still often saw only `2/2` fresh AprilTag samples, so the `4-of-5` filtered arrival gate never triggered even though `apriltag_z` was already below threshold.
- Behavior: default `color_profile` and `depth_profile` are restored from `640,480,15` to `640,480,30` while keeping the widened stale timeout and AprilTag QoS/thread changes.
- Rollback: restore both defaults to `640,480,15` if UVC protocol errors or USB timing instability become the dominant issue again.

## 2026-05-22 02:05 CST

### AprilTag wall-jump arrival continuity

- Changed `src/apriltag_ros/cfg/tags_36h11.yaml`.
- Changed `src/auto_nav_pkg/config/obstacle_waypoints.yaml`.
- Changed `src/auto_nav_pkg/config/obstacle_waypoints_mirror.yaml`.
- Reason: field log showed `wall_jump_prep` already had `apriltag_z=0.313m <= 0.470m`, but the last tag sample aged to about `0.61s` and was discarded by `apriltag_arrival_stale_timeout_sec=0.30`, so the crouch-to-stand transition never triggered.
- Behavior: AprilTag uses `qos_profile=sensor_data` and `detector.threads=2`; `wall_jump_prep` stale timeout is widened from `0.30s` to `0.80s` so short tag dropouts can still satisfy the existing filtered arrival logic.
- Rollback: restore `qos_profile=default`, `detector.threads=1`, and `apriltag_arrival_stale_timeout_sec=0.30`.

## 2026-05-22 01:45 CST

### RealSense stream-rate rollback

- Changed `src/auto_nav_pkg/launch/obstacle_race_visual_supervisor.launch.py`.
- Reason: field logs showed repeated `UVCIOC_CTRL_QUERY Protocol error` from `realsense2_camera_node` during obstacle runs; first rollback step is to reduce default stream rate and recover timing margin on the USB/device path.
- Behavior: default `color_profile` and `depth_profile` are now `640,480,15` instead of `640,480,30`.
- Rollback: restore both defaults to `640,480,30` after the camera link is confirmed stable.

## 2026-05-03 00:12:26 CST

### YOLO relative navigation trigger window

- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Set `target_class_name` to `montant` because `/yolo/detections_3d` echo showed the obstacle pole class is `montant`.
- Raised `trigger_distance_m` from `1.0` to `1.25` because `/yolo/detections_3d` was measured at about `1.1-1.4Hz`, with occasional long gaps, and the pole drops out around `0.75m`.
- Rollback: set `target_class_name: ""` and `trigger_distance_m: 1.0` if YOLO rate becomes stable or the class name changes.

### Goal topic QoS compatibility

- Changed `src/auto_nav_pkg/auto_nav_pkg/nav_executor.py`.
- Changed `src/quadruped_step_mapper/src/cmd_vel_to_serial_node.cpp`.
- Switched `/goal_pose` subscribers to volatile durability so they can receive goals from local task nodes, RViz, and Nav2 tooling without durability mismatch warnings.
- Rollback: restore transient local durability only if late-joining goal replay is required again.

### Runtime diagnostics

- Changed `src/auto_nav_pkg/auto_nav_pkg/yolo_relative_nav.py`.
- Added startup feedback fields for `target_class`, `min_score`, trigger threshold, and axis.
- Expanded candidate rejection feedback to include class, score, and axis checks.

## 2026-05-03 00:23:32 CST

### In-place rollback comments

- Added timestamped rollback comments directly at the modified `yolo_relative_nav.py` QoS, YOLO candidate filtering, and route-mode fixed-step gating code.
- Added a timestamped rollback comment directly at the `cmd_vel_to_serial_node.cpp` fixed-step runtime gating condition.
- Reason: keep the changelog as an index, while preserving the concrete rationale beside the code that changed.

## 2026-05-03 00:31:00 CST

### YOLO relative navigation heartbeat diagnostics

- Changed `src/auto_nav_pkg/auto_nav_pkg/yolo_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Added `status_heartbeat_sec`, default `2.0`, so `/yolo_relative_nav/state` and `/yolo_relative_nav/feedback_log` keep publishing while the node is alive.
- Heartbeat includes current state, trigger status, waypoint index, YOLO message age, raw detection count, candidate count, and nearest candidate summary.
- Reason: field logs showed `yolo_relative_nav` was subscribed, but late `ros2 topic echo` subscribers saw no event logs while the node was waiting.
- Rollback: set `status_heartbeat_sec: 0.0`.

## 2026-05-03 01:05:57 CST

### Standalone HSV orange pole detector

- Added `src/auto_nav_pkg/auto_nav_pkg/orange_pole_detector.py`.
- Added `src/auto_nav_pkg/config/orange_pole_detector.yaml`.
- Added `src/auto_nav_pkg/launch/orange_pole_detector_test.launch.py`.
- Added the `orange_pole_detector_node` console script and required image dependencies.
- Reason: CPU-only mini PC reaches about 95% CPU with YOLO alone; the right-angle pole obstacle can be localized with a much cheaper orange HSV mask plus D435 aligned depth.
- Behavior: finds all orange vertical candidates, computes depth for each, selects the nearest pole, publishes distance/pose/trigger/debug image, and logs distance, pixel position, candidate count, and trigger state.
- Rollback: do not launch `orange_pole_detector_test.launch.py`; continue using the existing YOLO launch and `yolo_relative_nav`.

## 2026-05-03 01:29:27 CST

### Orange pole depth estimate refinement

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_detector.py`.
- Changed `src/auto_nav_pkg/config/orange_pole_detector.yaml`.
- Added `depth_mask_erode_px`, `depth_percentile`, and `distance_bias_m`.
- Reason: field logs showed the HSV pole distance was stable but biased by about `7-8cm`; thin poles can mix background/floor depth at mask edges.
- Default behavior now erodes the depth mask and uses the near-side 25th percentile instead of the full-mask median.
- Rollback: set `depth_mask_erode_px: 0`, `depth_percentile: 50.0`, and `distance_bias_m: 0.0`.

## 2026-05-03 01:40:08 CST

### HSV orange pole route integration

- Added `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Added `src/auto_nav_pkg/launch/orange_pole_relative_nav_bringup.launch.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Changed `src/auto_nav_pkg/setup.py`.
- Reason: connect the lightweight HSV+D435 orange pole detector to the existing six relative waypoint route while keeping the YOLO route node available for rollback.
- Behavior: starts by publishing `[0,1,1]`, waits for `/orange_pole_detector/trigger` or nearest distance `<=1.25m`, switches route mode to `6`, disables the legacy fixed mode-6 serial override during navigation, then sends all six relative `/goal_pose` waypoints from current `/Odometry`.
- Rollback: do not launch `orange_pole_relative_nav_bringup.launch.py`; use `obstacle_race_bringup.launch.py enable_yolo_relative_nav:=true`.

## 2026-05-03 02:08:37 CST

### Measured pole route points and lidar-only waypoint file

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Changed `src/auto_nav_pkg/config/orange_pole_detector.yaml`.
- Added `src/auto_nav_pkg/config/pole_race_lidar_waypoints.yaml`.
- Reason: `src/点位.txt` contains measured Odometry poses at an about `1.35m` pole-start baseline; the old six rough offsets did not encode the measured turn poses.
- Behavior: HSV route now uses measured relative offsets from point 1 to points 2-12, measured yaw values, and direct mode-6 spin overrides on the short turn segments. The lidar-only YAML uses the same measured points as absolute Point-LIO waypoints for `obstacle_manager`.
- Rollback: restore the old six-offset `waypoint_offsets_xy`, set `goal_yaw_mode: "current"`, clear the new waypoint yaw/direct-override arrays, and set HSV trigger distance back to `1.25`.

## 2026-05-03 02:49:46 CST

### Updated required pole route from new point log

- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Changed `src/auto_nav_pkg/config/orange_pole_detector.yaml`.
- Rebuilt `src/auto_nav_pkg/config/pole_race_lidar_waypoints.yaml`.
- Reason: `src/点位.txt` was updated with a new 15-point measured route where point 1 is the start point, and every point must be reached.
- Behavior: HSV relative navigation now uses 14 measured offsets from point 1 to point 15, checks yaw at every point, and uses mode-6 direct step override for the short large-yaw turn points. The lidar-only YAML now contains all 15 measured points with `allow_goal_pass_through: false` on every point because the route is close to the obstacles.
- Rollback: restore the previous 11-target arrays and previous `pole_race_lidar_waypoints.yaml` entry from the 2026-05-03 02:08 changelog section.

## 2026-05-03 03:31:30 CST

### HSV route anchor and yaw-tolerance cleanup

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Changed `src/auto_nav_pkg/config/orange_pole_detector.yaml`.
- Reason: field logs showed each waypoint was being generated from the current pose, so arrival error accumulated; normal `/goal_pose` points also stalled near the target when yaw error was outside tolerance.
- Behavior: the HSV route now locks a trigger-time route anchor, converts the measured adjacent deltas into cumulative offsets, and sends every target as `anchor + cumulative_offset`. Normal forward points are position-only; short turn points still use mode-6 direct step override and yaw tolerance. The trigger threshold is advanced to `1.45m` to reduce anchor lag on low-rate HSV/depth frames.
- Rollback: set `route_reference_mode: "current_pose"`, restore `waypoint_use_yaw_tolerance` to all `true`, restore `yaw_tolerance_deg: 6.0`, restore the previous arrival tolerances, and set both HSV trigger distances back to `1.35`.

## 2026-05-03 03:48:14 CST

### Direct-turn approach gate

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Reason: field logs showed `waypoint=4/14` entering `DIRECT_STEP_NAVIGATING` while still `0.56-0.80m` from the target; direct step override was spinning instead of approaching the turn point.
- Behavior: direct-turn waypoints now publish `/goal_pose` first and only arm mode-6 direct override when distance is within `0.35m`. If the spin drifts beyond `0.50m`, the node clears direct override and re-publishes the goal for recovery. Turn yaw tolerance is loosened to `15deg`.
- Rollback: set `direct_step_start_distance_m: 99.0` for the old immediate direct-override behavior, or disable direct-turn overrides with `waypoint_use_direct_step_override` all `false`.

## 2026-05-03 04:04:00 CST

### Restore HSV offsets from point log and loosen arrival

- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Reason: a manual edit changed the `2->3` measured offset from `dy=0.030` to `dy=0.022`; field tests also showed the robot needed about `7cm` of manual forward correction before the route accepted arrival.
- Behavior: restored the HSV offset array from current `src/点位.txt`, specifically `2->3=(0.028, 0.030)`, and set all HSV route arrival tolerances to `0.30m`.
- Rollback: restore `waypoint_arrival_tolerances` to `0.22m` if the route starts cutting points too early.

## 2026-05-03 04:10:40 CST

### Forward-compensated HSV target offsets

- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Reason: widening arrival tolerance to `0.30m` made turn-mode transitions happen too early. The observed `7cm` correction is along the robot's current forward direction, so it should be encoded in target offsets instead of arrival tolerance.
- Behavior: recomputed HSV `waypoint_offsets_xy` from current `src/点位.txt` after shifting each target point `0.07m` along `yaw+90deg` in the `camera_init` frame. Restored arrival tolerances to `0.22m` and tightened direct override arming to `0.24m`.
- Rollback: restore the 2026-05-03 04:04 raw `src/点位.txt` offset array and set `direct_step_start_distance_m: 0.35`, `direct_step_recover_distance_m: 0.50`.

## 2026-05-03 11:28:49 CST

### HSV close-range mode 6 anti-stall

- Changed `src/quadruped_step_mapper/config/cmd_vel_to_serial.yaml`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Reason: field logs showed near-goal mode 6 serial steps dropping to about `0.08-0.12`, which can leave the robot standing until pushed by hand. Some near-goal self-spin also came from `nav_executor` final yaw control even though HSV route arrival is position-owned.
- Behavior: nonzero mode 6 outputs are now floored to `0.18` norm while zero-stop remains zero, and `nav_executor.use_final_yaw_control` is disabled for this shared route config.
- Rollback: remove mode `6` from `min_output_step_norm_override_modes`, remove `0.18` from `min_output_step_norm_override_norms`, and set `use_final_yaw_control: true`.

## 2026-05-03 11:54:39 CST

### Recomputed route from updated point log

- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Changed `src/auto_nav_pkg/config/pole_race_lidar_waypoints.yaml`.
- Reason: `src/点位.txt` was updated with corrected measured Odometry poses.
- Behavior: HSV route offsets and waypoint yaw values now come from the new 15-point log. The HSV route keeps the existing `0.07m` forward compensation along `yaw+90deg`; the lidar-only YAML was refreshed with the raw absolute measured points.
- Rollback: restore the `waypoint_offsets_xy`, `waypoint_yaws_deg`, and `pole_race_lidar_waypoints.yaml` values from the 2026-05-03 04:10 changelog state.

## 2026-05-03 12:13:47 CST

### Corrected route point 6 and offsets

- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Changed `src/auto_nav_pkg/config/pole_race_lidar_waypoints.yaml`.
- Reason: the earlier point log accidentally duplicated point 6 from point 5; the current `src/点位.txt` now contains the corrected point 6 at `x=-0.6808, y=2.3989, yaw=62.4deg`.
- Behavior: HSV `waypoint_offsets_xy` and `waypoint_yaws_deg` were recomputed from the corrected 15-point log with the existing `0.07m` forward compensation. The lidar-only waypoint YAML now uses the corrected raw absolute point 6, removing the previous zero-length `5->6` segment.
- Rollback: restore the `waypoint_offsets_xy`, `waypoint_yaws_deg`, and `pole_03_entry` values from the 2026-05-03 11:54 changelog state.

## 2026-05-03 13:59:52 CST

### Earlier direct-turn arming

- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Reason: field logs from `orange_pole_relative_nav_bringup` showed turn waypoints 2/4/6 sitting around `0.34-0.36m` from target with large heading error. With `direct_step_start_distance_m=0.24`, `pose_controller` tried to align heading first, which looked like long in-place spinning before the next point.
- Behavior: `direct_step_start_distance_m` is now `0.36m`, and `direct_step_recover_distance_m` is now `0.55m`, so short turn waypoints enter mode-6 direct step earlier while still preventing far-away direct spins above about half a meter.
- Rollback: set `direct_step_start_distance_m: 0.24` and `direct_step_recover_distance_m: 0.35`.

## 2026-05-04 00:45:19 CST

### Split forward-point arrival from close-range turn alignment

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/auto_nav_pkg/nav_executor.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Reason: field tests still showed two coupled failures: forward HSV waypoints could stop about `0.20m` short and wait for a manual push, while short turn waypoints near points `2/4/6/...` would pure-spin because `pose_controller` refused to translate until heading error dropped below `35deg`.
- Behavior: `orange_pole_relative_nav` now latches route start until both Odometry and a fresh nearest-pole snapshot are available, uses a narrow segment-end/pass-through corridor for non-turn forward waypoints, and decouples direct-turn approach yaw from final arrival yaw. `nav_executor` now keeps a capped forward component for close goals within `0.45m` when heading error is large but still below `110deg`, so short turn approaches arc in instead of freezing into pure in-place spin.
- Rollback: set `forward_waypoint_pass_through_enabled: false`, set `require_fresh_pole_for_route_start: false`, set `close_goal_relaxed_heading_enabled: false`, and restore `orange_pole_relative_nav.py` / `nav_executor.py` to the 2026-05-03 13:59:52 behavior.

## 2026-05-04 01:08:46 CST

### Replace close turn goals with signed turn primitives

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Reason: even after relaxing close-goal heading alignment, the direct-turn waypoints were still fundamentally modeled as tiny `/goal_pose` targets. That let `pose_controller` and shortest-angle yaw math keep competing with the intended turn direction, which is why field runs could still show long spins or reverse-direction spins at the turn points.
- Behavior: any waypoint with `waypoint_use_direct_step_override=true` now defaults to a single-direction turn primitive instead of a goal-approach phase. The primitive starts from the robot's actual current pose, keeps publishing the configured mode-6 override every control cycle, accumulates only commanded-direction yaw progress, and finishes on `yaw progress + end corridor/dist + min duration`, with a dedicated `6.0s` turn timeout. The old `/goal_pose`-then-direct-step path is still available behind `turn_waypoint_use_motion_primitive=false`.
- Rollback: set `turn_waypoint_use_motion_primitive: false`, remove the new `turn_waypoint_*` parameters, and restore `orange_pole_relative_nav.py` to the 2026-05-04 00:45:19 behavior.

## 2026-05-04 01:22:01 CST

### Replace turn-in-place primitive with arc-turn plus exit-forward

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Reason: the first signed turn primitive still used a single turn-only override until both yaw and distance were satisfied. In field logs that could still look like long in-place spinning when the override changed heading but did not eat enough forward distance, especially on the transition right after waypoint 1.
- Behavior: turn primitives now run in two phases. Phase 1 is a signed arc turn using both-positive asymmetric step norms, so the robot keeps a forward component while rotating instead of sitting and spinning in place. Phase 2 is a short equal-norm forward push that cleans up the remaining corridor error. Forward-point arrival to the next turn primitive now also skips the old `next_goal_delay_sec` gap, so the first point can hand off directly into the turn action without a dead stop.
- Rollback: remove the new `turn_waypoint_arc_*` and `turn_waypoint_exit_forward_*` parameters, delete the `ARC_TURN -> EXIT_FORWARD` phase logic, and restore `orange_pole_relative_nav.py` to the 2026-05-04 01:08:46 behavior.

## 2026-05-04 01:40:28 CST

### Add simple point-then-turn test mode

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Changed `src/quadruped_step_mapper/config/cmd_vel_to_serial.yaml`.
- Reason: the current HSV pole route is still mixing point tracking, turn geometry, and several serial-side posture corrections. For the next field pass we need a simpler baseline: each forward waypoint must be truly reached, each turn waypoint must only rotate in place to the configured yaw, and no extra path/yaw correction layers should interfere.
- Behavior: `orange_pole_relative_nav` now supports a temporary `turn_waypoint_simple_in_place_enabled` mode. When enabled, direct-turn waypoints stop acting like arc+forward primitives and instead keep publishing the configured mode-6 opposite-sign override until the requested signed yaw change is completed, then immediately advance to the next forward waypoint. The HSV route config also disables forward pass-through, disables `nav_executor` close-goal relaxed heading, and disables serial-side path lateral/yaw/crouch corrections plus same-direction turn shaping.
- Rollback: set `turn_waypoint_simple_in_place_enabled: false`, restore `forward_waypoint_pass_through_enabled: true`, restore `close_goal_relaxed_heading_enabled: true`, and revert the correction flags in `src/quadruped_step_mapper/config/cmd_vel_to_serial.yaml` to their previous `true` values.

## 2026-05-04 01:53:00 CST

### Push HSV waypoint 5 forward by 20cm along body heading

- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Reason: field runs showed the current HSV route around waypoint `5/14` still clipping too close to the pole. The requested change was to move waypoint 5 another `0.20m` forward along that point's body heading while keeping later absolute waypoints unchanged.
- Behavior: waypoint 5 uses `yaw=62.4deg`, and the route comments define body-forward as global `yaw+90deg`, so the applied global delta is approximately `(-0.177, +0.093)m`. In the adjacent-offset list, segment `4->5` was updated from `(-1.317, 0.669)` to `(-1.494, 0.762)`, and segment `5->6` was updated from `(0.121, 0.247)` to `(0.298, 0.154)` so only waypoint 5 moves while waypoint 6 and all later absolute targets stay in place.
- Rollback: restore segment `4->5` to `(-1.317, 0.669)` and segment `5->6` to `(0.121, 0.247)`.

## 2026-05-04 02:11:00 CST

### Add separate HSV body-relative route test mode

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Added `src/auto_nav_pkg/config/orange_pole_body_relative_test.yaml`.
- Added `src/auto_nav_pkg/launch/orange_pole_body_relative_nav_bringup.launch.py`.
- Reason: the next experiment needs a separate route flavor that keeps the current HSV trigger and turn-yaw logic, but lets waypoints after point 2 be expressed as displacements relative to the robot's current body heading instead of the route anchor / global frame.
- Behavior: `orange_pole_relative_nav` now accepts optional `body_relative_waypoint_indices` plus matching `body_relative_waypoint_offsets_fl` pairs, where each pair is `[forward_m, left_m]` in the robot body frame at the moment that waypoint is sent. A dedicated launch file now loads an extra override YAML on top of `obstacle_race_system.yaml`, so the body-relative experiment stays separate from the main HSV route. The provided override keeps waypoints `1`, `2`, and `14` on the base absolute route and leaves waypoints `3..13` as zero placeholders for manual filling.
- Rollback: launch the original `orange_pole_relative_nav_bringup.launch.py`, remove the new body-relative YAML/launch, and ignore the new `body_relative_waypoint_*` parameters.

## 2026-05-04 14:45:00 CST

### Gate waypoint 2 turn on fresh pole depth

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/orange_pole_body_relative_test.yaml`.
- Reason: in the body-relative test flow, waypoint `2/14` should not rotate immediately after waypoint 1 completes. The requested behavior is to wait until the latest fresh orange-pole depth is within `0.30m`, then start the original waypoint-2 turn.
- Behavior: `orange_pole_relative_nav` now accepts optional `turn_waypoint_pole_distance_gate_indices` plus matching `turn_waypoint_pole_distance_gate_thresholds_m`. When a turn waypoint is listed there, the turn primitive enters `TURN_PRIMITIVE_WAIT_POLE_DISTANCE` and does not publish rotation override until a fresh `nearest.distance_m` falls below the configured threshold. The body-relative test YAML now uses this for waypoint `2`.
- Rollback: remove `turn_waypoint_pole_distance_gate_indices` / `turn_waypoint_pole_distance_gate_thresholds_m` from the test YAML and ignore the new `turn_waypoint_pole_distance_gate_*` parameters.

## 2026-05-04 15:05:00 CST

### Add switchable body-relative turn yaw mode

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/orange_pole_body_relative_test.yaml`.
- Reason: the body-relative displacement experiment still used absolute global waypoint yaws for turn points, which mixed local displacement control with global heading targets. The next test needs turn points to optionally use signed yaw deltas relative to the robot's current body heading, while still preserving the original absolute-yaw route as a selectable fallback.
- Behavior: `orange_pole_relative_nav` now accepts `turn_waypoint_yaw_mode` with two options: `global_waypoint` keeps the existing `waypoint_yaws_deg` behavior, and `body_relative_delta` uses `body_relative_turn_waypoint_indices` plus `body_relative_turn_deltas_deg` to define signed turn deltas in degrees at each listed turn waypoint. In body-relative mode, the actual target yaw is computed from the robot's current heading at the moment the turn starts, after any pole-distance gate. The test YAML defaults to `body_relative_delta` and preloads the original route's turn deltas so field tuning can still focus on displacement values.
- Rollback: set `turn_waypoint_yaw_mode: "global_waypoint"` in the test YAML, remove the `body_relative_turn_waypoint_*` parameters, and ignore the new body-relative turn-yaw branch in `orange_pole_relative_nav.py`.

## 2026-05-04 15:18:00 CST

### Split forward and turn route modes for body-relative test

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/orange_pole_body_relative_test.yaml`.
- Reason: the body-relative test route was still inheriting a single `route_mode=6`, so even ordinary forward segments were entering the same down-slope / turn gait as the turn primitives. The requested behavior is normal walking on forward segments and mode-6 only while turning.
- Behavior: `orange_pole_relative_nav` now accepts optional `forward_route_mode` and `turn_route_mode`. If they are not set, the node falls back to the legacy single `route_mode` behavior. The body-relative test YAML now sets `forward_route_mode: 0` and `turn_route_mode: 6`, so normal point-tracking publishes mode `0` while direct overrides and turn primitives publish mode `6`.
- Rollback: remove `forward_route_mode` / `turn_route_mode` from the test YAML and let both behaviors fall back to the legacy `route_mode`.

## 2026-05-04 15:40:00 CST

### Make optional array parameters safe when empty or omitted

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Reason: the body-relative test YAML intentionally disables some optional features with empty arrays such as `turn_waypoint_pole_distance_gate_indices: []`. Under ROS 2 Humble, these typed array parameters can remain uninitialized, and direct `.value` reads caused `orange_pole_relative_nav_node` to die at startup with `ParameterUninitializedException`.
- Behavior: optional array parameters now use a safe accessor that treats uninitialized arrays as empty lists. This covers waypoint yaw arrays, direct-step arrays, body-relative waypoint arrays, body-relative turn arrays, pole-distance gate arrays, and per-waypoint arrival tolerances. The node now starts cleanly whether those arrays are omitted, populated, or explicitly set to `[]`.
- Rollback: revert the safe optional-array accessor and require every typed array parameter to be fully initialized in YAML before node startup.

## 2026-05-04 15:58:00 CST

### Keep only waypoint 1 in normal gait and rewrite body-relative turn sequence

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/orange_pole_body_relative_test.yaml`.
- Reason: the body-relative test should keep only the very first forward waypoint in the normal gait, then switch all later forward segments and all turn primitives to mode `6` for speed. The field description also replaced the five interior body-relative turn angles with a right-angle wrap pattern.
- Behavior: `orange_pole_relative_nav` now accepts `first_forward_route_mode`, which overrides `forward_route_mode` only for waypoint `1/14`. The body-relative test YAML now uses `first_forward_route_mode: 0`, `forward_route_mode: 6`, and `turn_route_mode: 6`. For body-relative yaw control, waypoint `2/14` and final waypoint `14/14` keep their existing turn deltas, while waypoints `4/6/8/10/12` now use `cw45`, `ccw90`, `cw90`, `ccw90`, `cw90`.
- Rollback: remove `first_forward_route_mode`, set `forward_route_mode` back to the desired single forward gait, and restore the previous `body_relative_turn_deltas_deg` list.

## 2026-05-04 16:06:00 CST

### Let body-relative turn deltas decide turn override direction

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Reason: after switching the body-relative test route to a new signed turn-angle sequence, some turn waypoints inherited old `waypoint_direct_step_left_norms/right_norms` signs from the original route. The old code treated that sign mismatch as a fatal configuration error and stopped the whole route at the first conflicting waypoint.
- Behavior: when `turn_waypoint_yaw_mode=body_relative_delta`, the turn direction now comes from the sign of `body_relative_turn_deltas_deg`, not from the legacy sign baked into `waypoint_direct_step_left_norms/right_norms`. The configured left/right values are now used only as magnitude templates, and the runtime rewrites them to match the requested clockwise or counter-clockwise direction for both turn primitives and the fallback direct-override path.
- Rollback: restore the old sign-consistency check in `_start_turn_primitive()` and stop rewriting direct-step overrides from `body_relative_turn_deltas_deg`.

## 2026-05-04 16:18:00 CST

### Ramp startup override from zero instead of jumping to full stride

- Changed `src/auto_nav_pkg/auto_nav_pkg/orange_pole_relative_nav.py`.
- Changed `src/auto_nav_pkg/config/obstacle_race_system.yaml`.
- Reason: field runs showed the pre-trigger startup override jumping directly from `[0,0,0]` to `[0,1,1]`, which made the first forward motion too abrupt.
- Behavior: `orange_pole_relative_nav` now supports `startup_ramp_duration_sec`. While waiting for trigger, the startup step override linearly ramps `left_norm/right_norm` from zero to the configured `startup_left_norm/startup_right_norm` over that duration. Both the YOLO and HSV race configs now use `startup_ramp_duration_sec: 1.0`.
- Rollback: set `startup_ramp_duration_sec: 0.0` to restore the old immediate `[mode,left,right]=[0,1,1]` behavior.
