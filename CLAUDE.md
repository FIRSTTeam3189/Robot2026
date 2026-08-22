# Robot2026 — Team Notes for Claude

FRC competition robot code (C++, WPILib + GradleRIO + PathPlannerLib). This file
captures context that isn't obvious from reading the code alone — architecture
decisions, known bugs, open issues, build quirks, and collaboration norms
established through actual investigation. Keep it updated as things change;
strike through or delete items once they're resolved so it doesn't rot into
noise.

## Build environment

- GradleRIO 2026.2.1 requires **JDK 17+**. The default `JAVA_HOME`/`PATH` java
  in this environment resolves to JDK 11, which fails configuration
  (`Dependency requires at least JVM runtime version 17`).
- A working JDK (OpenJDK 19.0.2) is available at
  `/c/Users/showb/.jdks/openjdk-19.0.2`. To build from a shell here:
  ```bash
  export JAVA_HOME="/c/Users/showb/.jdks/openjdk-19.0.2"
  export PATH="$JAVA_HOME/bin:$PATH"
  ./gradlew.bat compileFrcUserProgramLinuxathenaDebugExecutableFrcUserProgramCpp
  ```
- `compileCpp` alone is **ambiguous** (Gradle will list ~8 candidate tasks) —
  use the full `compileFrcUserProgramLinuxathenaDebugExecutableFrcUserProgramCpp`
  task name for a real roboRIO-target compile check.
- Don't rebuild reflexively after every edit — the user has explicitly asked
  not to rebuild unprompted in at least one session. Offer, don't just do it,
  unless verifying a nontrivial change before a stated deadline.

## Repo history note

The earliest commit in this repo (`69f9ddd`, "stuff") is a **bulk import of
prior seasons' code** — git history doesn't meaningfully go back further than
that. Don't expect `git log -S` / `git blame` to explain the origin of old
constants; they may predate this repo entirely.

## Architecture overview

- `RobotContainer` is the composition root: constructs all subsystems, wires
  button bindings (`ConfigureDriverBindings()` for the main driver "Bill" on
  `m_bill`, `ConfigureTestBindings()` for a secondary/test controller
  `m_test`), registers PathPlanner named commands, and builds the auto
  chooser.
- **Dual robot config**: `constants/swerve/SwerveModule.h` (and others) branch
  on `#ifdef ROBOTConstants2026V2` — two full sets of constants for two
  different physical robots live side by side in the same file. Easy to edit
  one branch and forget the other; check both when tuning drivetrain
  constants.
- **Swerve drivetrain** (`subsystems::swerve::Drive` + `util::SwerveModule`):
  4x CTRE TalonFX (drive+angle) + CANcoder per module, Pigeon2 gyro,
  `frc::SwerveDrivePoseEstimator<4>` wrapped by `PoseEstimatorHelper`.
  PathPlanner is wired in via `AutoBuilder::configure` in `Drive`'s
  constructor.
- **Vision/pose fusion** — see dedicated section below.
- **Shooter**: `Turret` (rotation, aims at hub), `Hood` (articulation angle),
  `Flywheel` (SparkFlex velocity control). Behavior is driven by a
  `constants::ShooterState` enum (`Automatic`, `Fixed`, `Reset`, `Manual`,
  `HoodAndInterpAuto`, `ManualTurretOnly`) dispatched every loop in
  `Robot::RobotPeriodic` — the enum + switch is the actual source of truth for
  "what is the shooter doing right now," not any single subsystem's own
  state.
- **SysId**: already wired up for `Turret`, `Hood`, `Intake`, `Loader` — but
  **not** for `Drive`/`SwerveModule`. If characterizing the drivetrain, follow
  the existing pattern in `Turret.h`/`Turret.cpp` rather than inventing a new
  one.
- Constants live under `constants::` split by subsystem
  (`constants/swerve/SwerveModule.h`, `constants/swerve/SwerveDrive.h`,
  `constants/Vision.h`, `constants/Automatic.h`, `constants/Global.h`, etc).

## Vision / pose fusion architecture

`PoseEstimatorHelper` wraps the WPILib `SwerveDrivePoseEstimator<4>`. Four
things feed into it:

| Source | Trust (x, y, θ std dev) | Notes |
|---|---|---|
| Wheel encoders + gyro | `kEncoderTrustCoefficients` — 0.1/0.1/0.1, fixed all match | Continuous integration via `UpdateWithTime` every `Periodic()`. This is the backbone; everything else is a periodic correction on top. |
| PhotonVision (3 cams: front/left/right turret) | `kPhotonVisionTrustCoefficients` base 0.5/0.5/10.0, grows with tag distance | Filtered by ambiguity (`kMaxAmbiguity`) and distance/jump thresholds — **see open issue below, thresholds are currently useless**. |
| QuestNav | `kQuestnavTrustCoefficients` — currently 0.01/0.01/1.0 | **No filtering at all** before fusion — every frame gets fused unconditionally, unlike Photon. |
| "Main Vision" | `kMainVisionTrustCoefficients` | **Dead** — see vestigial code section. Only used as the estimator's throwaway initial value. |

**Rotation is always overwritten to the gyro's estimate for every vision
source, by design** — `PoseEstimatorHelper::AddVisionMeasurement` deliberately
replaces whatever rotation a vision pose carries with the estimator's own
current rotation before fusing, because the team decided to fully trust the
gyro for heading and never let vision touch it. This is intentional, not a
bug — but see the timestamp bug below, which broke the *implementation* of
this intent.

**QuestNav mounting-offset direction — easy to get backwards:**
- `SetPose()` needs the **Quest's own pose** (physical device location +
  orientation), not the robot's. To reset it correctly: take the robot's
  target pose and compose it *forward* with the mount offset
  (`robotPose.TransformBy(mountOffset)`).
- The continuous fusion path (`Questnav::Periodic` → `AddVisionMeasurement`)
  needs the **robot's pose**, but the device reports its *own* pose. Compose
  with the *inverse* (`questPose.TransformBy(mountOffset.Inverse())`).
- Both directions must use the *same* underlying offset constants
  (`kQuestMountXOffset`/`YOffset`/`ZOffset`/`RollOffset`/`PitchOffset`/`YawOffset`
  in `Vision.h`) — currently all `0.0`, pending the Quest actually being
  physically mounted and measured.
- Why a wrong mounting rotation causes *translation* to be wrong too, not just
  heading: `SetPose()` establishes the device's entire internal SLAM
  coordinate frame. If the anchor rotation is wrong, the device's own x/y
  tracking axes are misaligned with the field from that point forward — not
  just a mislabeled heading. Re-anchoring with a fresh, correct `SetPose()`
  call fixes both; no amount of per-frame rotation math can fix a bad anchor
  after the fact.

## Bugs fixed this session (2026-07)

- **`Questnav` had an uninitialized pointer.** Constructor was
  `Questnav::Questnav() = default;` with a `PoseEstimatorHelper*` member never
  assigned — calling `AddVisionMeasurement` through it was undefined behavior.
  Fixed by adding a constructor parameter, mirroring how `Photon` receives its
  `PoseEstimatorHelper*`.
- **Rotation-drift bug in `PoseEstimatorHelper::AddVisionMeasurement`.** The
  "trust gyro, not vision, for rotation" logic stamped the vision measurement
  with `GetEstimatedPose().Rotation()` — the estimator's **current, live**
  rotation — onto a measurement being inserted at an **old** timestamp (vision
  latency). If the robot was actively rotating in that gap, WPILib's
  estimator interpreted the mismatch as a real gyro error and applied a
  phantom correction, which compounds especially badly with QuestNav (fires
  very frequently, zero filtering) during fast spins. Fixed by using
  `m_poseEstimator->SampleAt(timestamp).value_or(GetEstimatedPose()).Rotation()`
  instead — asks the estimator what rotation it *actually believed* at that
  historical instant, so the two values genuinely match and no phantom
  correction gets injected.
- **QuestNav mounting-offset plumbing added** (offsets still `0.0` pending
  physical mount — see above).

## Known open issues (not yet fixed)

- **PhotonVision bad-frame rejection is effectively disabled.**
  `kMaxDistance` and `kUseVisionMeasurementThreshold` in `Vision.h` are both
  set to `100.0` (meters) — the FRC field is ~16m — so neither guardrail ever
  actually triggers. A single glitched low-ambiguity-but-wrong-location Photon
  frame currently has nothing stopping it from being fused as a confident
  measurement.
- **`kModuleSpeedMultiplier` (currently `1.311`) is an unverified,
  possibly-stale empirical fudge factor** applied to the drive motor's
  closed-loop velocity setpoint. Leading hypothesis: compensates for velocity
  undershoot from a P-only closed loop with zero `kV`/`kS` feedforward, and/or
  FOC (`WithEnableFOC(true)`) voltage-utilization loss. It's been unchanged
  across 3+ years of different wheel radii/gear ratios, which supports it
  being a hardware/firmware-level constant rather than a per-robot mechanical
  correction. Needs a bench test (FOC on/off, measured vs. commanded speed) to
  verify it's still correct — ideally replaced with a real `kV`/`kS`
  characterization via SysId rather than a flat multiplier. **Do not
  recalibrate this without a driver practice window before the next
  competition** — it changes teleop top speed/feel, and the driver has
  adapted to the current (possibly wrong) behavior across 3 competitions
  already.
- **No CAN device fault/connection monitoring on the drivetrain.** Pigeon2,
  CANcoders, TalonFXs are configured once at startup; nothing in `Periodic()`
  checks connection/fault state, unlike the vision code which at least checks
  `IsConnected()`/`IsTracking()`.
- **`Drive::SetPose()` blindly retries `m_pigeon.SetYaw()` 10 times** with no
  status check — likely a workaround for a flaky CAN write that was never
  actually diagnosed.
- **Single-tag PhotonVision only** — always uses `GetBestTarget()`, never
  multi-tag pose estimation, which is more prone to the classic ambiguity
  "flip" than multi-tag solves.
- **`kWheelCOF`** (coefficient of friction) is defined in `SwerveModule.h` but
  never referenced anywhere — no traction/slip-aware acceleration limiting
  actually exists despite the constant being there.
- Duplicated constants across the `#ifdef ROBOTConstants2026V2` / `#else`
  branches risk drifting out of sync when only one is edited.

## Potential future improvements (not started)

- **Automatic hard-stop detection for turret zeroing.** The Circle-button
  manual re-zero (`RobotContainer.cpp`'s `runTurretLeftButton` →
  `Turret::ZeroTurret()`) drives the turret left at low power with the
  reverse soft limit disabled, and re-anchors the turret's position to
  `kMinimumAngleNegative` (-56°) the instant the driver releases the button —
  relying entirely on driver judgment to release once it's actually jammed
  against the true mechanical hard stop. No sensor confirms this. Idea:
  detect the hard stop automatically instead of requiring a manual release.
  Prefer **velocity-based stall detection** (watch `m_rotationVelocity` —
  already read every loop via `RefreshAllSignals()` — for sustained
  near-zero velocity across a few consecutive loops, with a small
  noise-tolerance band) over current-spike detection: the commanded power
  here is low (`-0.07`), so a stall current spike at that duty cycle would be
  small and easy to confuse with normal noise, whereas velocity directly
  measures "did it actually stop moving." Whatever detection method is used,
  pair it with a max-time safety timeout independent of the detection logic,
  so a sensor glitch or logic bug can't grind the mechanism into the hard
  stop indefinitely.

## Vestigial / dead code

- `kSyncBytes`, `kBaudRate`, `kBufferSize`, `kMainVisionTrustCoefficients`,
  `kMainVisionStdDevPerMeter` in `Vision.h` — scaffolding for a planned
  serial/UART-based coprocessor vision system (possibly the "rust vision"
  referenced in a stale comment in `PhotonVision.cpp`) that was never actually
  built. `kSyncBytes` is only ever referenced via throwaway
  `(void)constants::vision::kSyncBytes[0];` lines scattered across several
  files, apparently just to suppress an unused-variable warning.
- A large number of commented-out button bindings in `RobotContainer.cpp` —
  manual power control for intake/hood/flywheel/turret/kicker/loader on both
  controllers, fixed-angle test buttons, increment/decrement velocity
  buttons. If a control seems like it should work but doesn't, check here
  first before assuming it's a live bug.

## Collaboration norms

- Don't make changes that alter **teleop drive feel** without a driver
  practice window before the next competition — even changes that are
  objectively "more correct" (e.g. properly characterized feedforward
  replacing a flat multiplier) can hurt more than help if they land right
  before an event and throw off calibrated muscle memory. Changes isolated to
  autonomous-only code paths don't carry this risk.
- Confirm before touching pose-estimator/drivetrain safety-relevant logic;
  this is competition-critical code (see `git log` — robot has already
  competed on this codebase this season).
- When making C++ changes, verify with a real compile
  (`compileFrcUserProgramLinuxathenaDebugExecutableFrcUserProgramCpp`) when
  practical — this codebase has had real, previously-uncaught bugs (see
  above), so "looks right" isn't sufficient confidence on its own.
