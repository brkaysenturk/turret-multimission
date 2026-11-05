# Changelog

All notable changes to the Turret FCS (Autonomous Air Defense Turret Fire Control System) project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [3.0.0] - 2026-11-03

v3.0 Production Release standardizes the dual-subsystem monorepo combining Autonomous Air Defense (`turret_fcs`) and Cup Basket Challenge (`turret_basket`), unifies physical & Webots digital twin engagement timings, restores disturbance observer (DOB) support, and expands the STM32 FCS binary command frame to 8 bytes for dynamic flywheel speed control.

### Added

- **Cup Basket Challenge subsystem (`turret_basket/`):** Full standalone 2-stage hierarchical AI system (Cup Mouth Tracker + Neural Ballistics MLP policy `cup_ballistic_policy.npz`) felling darts into table-top cups across varying ranges and super-elevation angles (+21.8° to +22.6°).
- **Academic Basket Analysis Suite (`turret_basket/analysis/`):** 7 comprehensive academic figures and analysis scripts including RK4 ballistic trajectories, neural policy decisions, 24-table tolerance heatmaps, engagement timing profiles, and Optuna convergence plots.
- **Dynamic Flywheel Speed via 8-Byte Command Frame (`turret_fcs/protocol.py`, `turret_fcs/stm32/Core/Src/fcs_comm.c`):** Expanded PC-to-STM32 command frame from 7 bytes to 8 bytes (`[0xAA][pan i16][tilt i16][speed u8][state u8][checksum u8]`), enabling dynamic TB6612FNG PWM duty modulation without re-flashing STM32 firmware.
- **Disturbance Observer support (`turret_fcs/dob.py`):** Restored 1st-order Q-filter disturbance observer module for external disturbance compensation.
- **Webots Contact Dynamics Tuning:** Refined restitution (`bounce = 0.01`, `bounceVelocity = 0.5`) and angular damping (`0.35`) in `turret_basket.wbt` eliminating trampoline recoil in cup-landing mechanics.

### Changed

- **Version number:** Bumped to `3.0.0` in `pyproject.toml`, `turret_fcs.__version__`, and documentation.
- **Engagement timing synchronization:** Unified pre-spin, slew, settle, firing, and recovery timelines across physical STM32 firmware and Webots simulation controllers.
- **Standardized HUD layout:** Equalized tactical HUD layout, typography, reticle rendering, and color palettes across `turret_fcs` and `turret_basket`.

---

## [2.1.0] - 2026-10-25

v2.1 refines closed-loop tracking robustness, Kalman outlier recovery, short-dropout ego-motion bridging, STM32 UART frame resynchronization, and RL plant fidelity.

### Added

- **Candidate-replay Kalman outlier gate (`turret_fcs/kalman_filter.py`):** 150 px outlier measurements are buffered in `_outlier_history`. Single-frame YOLO spikes advance the filter via pure state prediction without corrupting velocity; when a new jump persists consistently for `reacquire_frames` (3 frames), the filter initializes at the first jump frame and replays subsequent candidate frames (`_predict_state` + `_correct_state`) to recover target velocity immediately.
- **Ego-compensated dropout prediction bridge (`turret_fcs/fcs_manager.py`):** During momentary target dropouts (`lost_counter <= max_lost_frames`), `FCSManager` continues updating 1st-order servo ego-motion shift (`_compute_ego_pixel_shift`) and advances `tracker.predict_only(dt, ego_dx, ego_dy)`, allowing the active controller (P or RL) to track the predicted trajectory seamlessly across short detection gaps.
- **Confidence-weighted target selection (`turret_fcs/target_selector.py`):** Spatial gate fallback and reticle-center target acquisition now weight squared pixel distance by detection confidence (`dist_sq / max(conf, 0.1)`).
- **Continuous-time dt-scaled EMA (`turret_fcs/p_controller.py`):** Angular error EMA filtering in `PController.compute_angles` now scales dynamically with frame `dt` (`alpha_eff = 1.0 - (1.0 - alpha) ** (dt / nominal_dt_s)`).
- **High-fidelity RL plant & curriculum pipeline (`turret_fcs/rl/`):** Added 2nd-order actuator inertia, 0.10 deg UART quantization residue accumulation, harmonic YOLO bbox jitter, multi-stage curriculum training (`train_rl.py`), and resonance-gated (`flip_rate + jerk`) Optuna TPE evaluation (`optimize_rl.py`).

### Changed

- **Complete SSOT constant centralization (`turret_fcs/config.py`):** Added `default_fps`, `backlash_deg`, `yolo_imgsz`, `tracker_config`, and RL reward weights (`lambda_u`, `mu_du`, `progress_gain`, `error_gain`, `bullseye_reward`, `bullseye_threshold_norm`) to `CONFIG`; removed duplicate `BallisticsConfig.spinup_time_s` and bound all runtime, Webots, and dataset capture modules to `CONFIG`.
- **CI ruff integration (`.github/workflows/ci.yml`, `pyproject.toml`):** Added `ruff==0.8.4` to `[project.optional-dependencies].test` and enabled `python -m ruff check` on both Linux and Windows CI runners.

### Fixed

- **STM32 UART sliding-window resynchronization (`stm32/turret_main/Core/Src/fcs_comm.c`):** On checksum mismatch, `FCS_Comm_RxCallback` now shifts `rx_buffer` to the next `0xAA` header byte instead of discarding the entire 6-byte window, preventing valid command loss after a single corrupted byte.
- **Initial LiDAR range-jump false positive (`turret_fcs/fcs_manager.py`):** Guarded `range_jump` detection with `bool(self._range_history)` so the first valid LiDAR reading is not rejected when it differs from the 2.0 m startup default by more than `jump_threshold_m`.

---

## [2.0.0] - 2026-10-24

v2.0 is the major consolidation of every post-v1.3 change. It bundles the previous audit work (repo-wide audit, production P and pure RL, Kalman redesign, firmware hardening) together with the protocol single-source refactor, the firmware E-stop and module headers, and the corrected figure labelling.

### Added

- **Protocol single source of truth:** `turret_fcs/protocol.py` and `stm32/turret_main/Core/Inc/protocol.h` now own the frame lengths, header bytes, angle scale and state codes. `fcs_comm.py`, `fcs_comm.c` and `luna_read.c` read these constants, so a one-sided change can no longer shift fields while still passing the checksum.
- **Latched hardware E-stop on B1:** the blue button (`PC13`/`EXTI13`, internal pull-up, 250 ms debounce) toggles a software latch. The first press cuts the launcher, parks the servos and ignores PC commands; the second press releases the latch. `EXTI15_10` and `USART2` NVIC lines are now part of `turret_main.ioc`, so a CubeMX regeneration keeps the PC command interrupt.
- **Firmware module headers:** `servo_driver`, `luna_read` and `flywheel_solenoid` gained module documentation headers, and `Launcher_Update` gained a safe `default` branch.
- **Debug/Release builds:** both configurations documented with their sizes in `docs/firmware.md`.
- **README diagram generator:** `analysis/comparison/generate_readme_diagrams.py` rebuilds the README architecture figures from the current module layout.
- **Weights release pinning:** `turret_fcs/weights.py` resolves `best.pt` and `yolov8n.pt` from the `v2.0` release assets.

### Changed

- **Version number:** set to `2.0.0` in `pyproject.toml`, `turret_fcs.__version__`, module headers, the architecture diagram and the field-test analysis captions. The RL artifact names (`turret_ppo_v14.zip`, `turret_ppo_v14_policy.npz`) are unchanged because they name the trained model, not the release.
- **fig7/fig8 renamed:** `fig7_real_hardware_still_lock.png` and `fig8_real_hardware_moving_tracking.png` are now `fig7_sim_still_lock.png` and `fig8_sim_moving_tracking.png`. Both are generated by the closed-loop simulator in `analysis/comparison/plot_tuned_system.py`; the old names claimed real-hardware measurements. Every figure was regenerated from the current script.

### Removed

- **`docs/evidence/`:** the directory held two telemetry CSVs whose every row carried `control_mode = PID` (a pre-v1.3 schema), while the summary page reported metrics that do not match those files (`summary.md` quotes 0.42 deg pan RMS, 0.92 lock ratio; the CSV recomputes to 6.45 deg and 0.26). The folder was removed rather than kept as misleading evidence.

### Fixed

- **Test count documented:** the suite is 36 tests; `AGENTS.md` no longer quotes the stale 24.

> `v1.3` and `v2.0` are the only release tags between `v1.0` and this release.

---

## [v1.3] - 2026-10-21

### Added

- **Single Source of Truth (SSOT) Architecture (`turret_fcs/config.py`):**
  - Consolidated all optical, actuator, controller, kalman, lock, ballistics, and path configurations into a lightweight, pure-math leaf module.
  - Eliminated circular dependencies and heavy imports from config.
  - Added `ActuatorConfig` with exact hardware step limits (0.55 deg pan, 0.22 deg tilt) and servo deadband (0.10 deg).
- **Telemetry v2 Specification & Sidecar Metadata (`turret_fcs/telemetry.py`):**
  - Expanded telemetry schema from 25 columns to 40 columns including `schema_version`, `frame_id`, `latency_ms`, `vx_raw`, `vy_raw`, `innov_px`, `sigma_a`, `kp_eff_pan`, `kp_eff_tilt`, `db_eff_pan`, `db_eff_tilt`, `servo_pan_fb`, `servo_tilt_fb`, `sensor_temp_c`, and `lock_thr`.
  - Automatic JSON metadata sidecar (`.meta.json`) recording system configuration, git commit SHA, session timestamps, and full post-session metrics (FPS, lock ratio, tracking error median/IQR).
  - Tactical event sidecar (`_events.csv`) logging all state transitions, locks, fire authorizations, and mode switches with millisecond timestamps.
- **Redesigned Linear-Quadratic Regulator (LQR) Reward Formulation for RL:**
  - Formulated continuous state-action penalty reward: `r = -(e_norm^2 + lambda_u * u_norm^2 + mu_du * du_norm^2) + bullseye_bonus`.
  - Bayesian hyperparameter optimization with Optuna TPE over 20 fixed seeds using robust Median + IQR scoring.
  - Retrained 250,000 steps PPO policy (`turret_ppo_final.zip`) achieving +59.0% tracking accuracy improvement over baseline P controller in benchmark tests.
- **Automated Pytest Test Suite & Continuous Integration:**
  - Created 19 automated test cases in `tests/` covering SSOT consistency, path resolution, controller deadband behavior, Joseph form Kalman stability, FCSState firmware enum parity, aim-point slew feedforward, and full FCSManager smoke pipelines.
  - Added GitHub Actions workflow (`.github/workflows/ci.yml`) matrix-testing Python 3.10, 3.11, and 3.12.
- **Exact 2D Gaussian Hit Probability (`turret_fcs/lock_manager.py`):**
  - Closed-form Marcum Q / Non-central chi-squared CDF (`scipy.stats.ncx2.cdf`) replacing heuristic attenuation.
  - Dynamic target angular radius computed directly from YOLO bounding box dimensions (`bw, bh`) and camera focal lengths.
  - Explicit separation of projectile flight time (`t_flight = d / v_proj`) and 80 ms transport delay with realistic weapon dispersion (`sigma_weapon = 1.25 deg`).
  - Extended dynamic hysteresis lock window (1.60° - 3.00° entry, 2.20° - 3.60° exit) with 10-frame stability filter.
- **Ballistic Firing Synchronization, Slew Feedforward & Lock Grace Period:**
  - Extended flywheel spin-up to 1.0s (`spinup_time_s = 1.00`), integrated direct aim-point slew feedforward (`u = delta_slew + u_P`) eliminating steady-state ramp lag, and ensuring servo achieves full super-elevation angle before solenoid release.
  - Added 0.75s post-firing lock grace period (`post_fire_grace_duration_frames = 22`), preventing false lock drops as turret returns to boresight.
- **Combat Declutter & Encapsulated Tactical HUD (`turret_fcs/turret_hud.py`):**
  - Encapsulated status banner, footer legend, and event logs into `TurretHUD` methods following SRP.
  - Automatically hides status panels, event logs, and FPS counters during fire authorization and firing cycle.
  - English tactical post-firing engagement feedback (`ENGAGEMENT COMPLETE / ENGAGEMENT SUCCESSFUL`).
- **HUD Color Discipline Decoupling:**
  - Decoupled tactical status banner colors from aiming reticle and target bounding boxes.
  - Reticle, target bounding box, and `TARGET` label remain strictly high-contrast Red during lock and firing sequences, while only the status banner updates through stage colors.

### Changed

- Synchronized Webots digital twin camera FOV (48.8 deg horizontal, 28.6 deg vertical) directly to `CONFIG.optics`.
- Replaced all raw numerical E-stop literals (3, 0) with `FCSState.ESTOP` and `FCSState.TRACKING`.
- Aligned solenoid actuation timing across documentation, config, and firmware to 80 ms (0.08 s).
- Clarified benchmark lock duration metrics: 1,061 frames corresponds to 13.16 seconds at 80.6 FPS.

- **Webots v1.3 Digital Twin Refactoring & High-Fidelity HAL Architecture (`webots/`):**
  - Replaced 8 legacy duplicated controller copies (`webots_fcs_manager`, `webots_pid_controller`, `webots_kalman_filter`, `webots_lock_manager`, `webots_rl_controller`, `webots_patrol_generator`, `webots_target_selector`, `webots_turret_hud`) with a single thin hardware abstraction layer importing production `turret_fcs` modules directly.
  - Implemented high-fidelity plant dynamics in `webots_hardware_interface.py`: 80 ms transport delay queue (`collections.deque(maxlen=3)`), 45 ms 1st-order servo lag (`tau = 0.045 s`), 1.20° mechanical backlash hysteresis, and 0.10° STM32 UART quantization.
  - Aligned physical world (`webots_turret_fcs.wbt`) parameters with v1.3 SSOT: camera FOV 48.8° (0.8517 rad), motor velocity 0.707 rad/s (40.5 deg/s rate limit), and mechanical gimbal limits (Pan ±75°, Tilt ±25°).
  - Implemented plant-accurate anti-windup incremental motor command integration (`cmd = curr + delta`) matching STM32 firmware behavior.
  - Synchronized firing sequence to pure simulation time (`sim_t = robot.getTime()`) with 1.0s spin-up, 80 ms solenoid pulse, 350 ms recover, and 1.5s EVAL window with 0.75s grace period; enforced lock requirement on `[Y]` fire key.
  - Connected 3D virtual referee directly to `KAFA_BLOGU` 3D orientation matrix eliminating coordinate projection ambiguity.
  - Added selectable field-aligned target flight profiles (0°/s static, 2-4°/s slow hover, 5-7°/s standard, 8-11°/s agile) to `balloon_mover.py`.
  - Upgraded Webots telemetry logger to Schema v2 with synchronized ground truth target coordinates and proper `fcs_state` logging.
  - Refactored kinematic mission and controller comparison scripts to test production `FCSManager` with explicit synthetic labeling.

### Fixed

- Fixed broken image references in `README.md` pointing to root instead of `docs/assets/`.
- Deleted duplicate analysis graphic `analysis/system_identification_analysis.png`.
- Updated `CameraThread.read()` to support optional `return_id=True` and zero-latency hardware frame IDs.
- Connected LiDAR temperature reading `temp_c` into real-time telemetry logging instead of discarding.

---

## [v1.2.0] - 2026-10-10

### Added

- Cumulative step accumulator in patrol generator (`min_dispatch_deg = 0.95 deg`) to prevent servo deadband stalling.
- Dynamic Kalman velocity deadband expanding from 45 px/s to 120 px/s based on ego-motion speed and target proximity.
- Tracking confidence hysteresis with dynamic thresholding (0.28 acquisition, 0.16 tracking persistence).
- Asymmetric elevation damping and vertical zero-jerk EMA filter to eliminate stick-slip limit cycles under 300g barrel inertia.
- Smith predictor action memory in PPO environment for sim-to-real latency alignment.
- Consolidated telemetry inspection suite under `analysis/logs/inspect_logs.py`.

---

## [v1.1.0] - 2026-09-20

### Added

- Type-1 Proportional control backbone with soft deadband for chattering-free servo response.
- Range gating protection filtering out background wall LiDAR reflections for errors > 3.0 deg.
- 11-dimensional action history PPO reinforcement learning agent with domain randomization.
- Reverse sliding-window binary UART parser for 115200 baud PC-STM32 telemetry.
- Multi-layer fail-safe architecture (300 ms STM32 heartbeat watchdog, 350 ms camera freeze guard, latched E-stop, 80 ms edge-triggered solenoid latch).
- Lissajous curve parametric air surveillance patrol mode (Pan: +/- 35 deg, Tilt: +/- 8 deg).
