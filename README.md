# Robot Physics Sim

A physics simulator for our FRC robot code. It compiles the **real robot code**
(from its own folder, nothing copied) together with simulated motors, sensors,
Limelights, the field, and game pieces. You drive it with a controller, watch it
in AdvantageScope, and run automated tests against it.

The sim never runs on the robot; it only replaces the hardware on a laptop.

## Set up on a new computer

1. **Install WPILib** for the season (it includes VS Code, AdvantageScope and Java 17).
   - **Windows:** also install *Visual Studio 2022 Build Tools* with the
     "Desktop development with C++" workload (the desktop C++ compiler).
   - **macOS:** run `xcode-select --install`.
   - **Linux:** install `build-essential`.
2. **Put the two folders side by side:**
   ```
   <any folder>/
     Rebuilt/            <- the robot code
     Physics SIM frc/    <- this sim
   ```
   Different name or place? Set `robotProjectDir` in `gradle.properties`, or set
   the `ROBOT_PROJECT_DIR` environment variable to the robot code folder.
3. **Open this folder in WPILib VS Code** and run **WPILib: Simulate Robot Code**.
   The first build downloads libraries, so be online.

If a terminal build says it needs Java 17, point `JAVA_HOME` at the WPILib JDK
(`C:/Users/Public/wpilib/<year>/jdk` on Windows, `~/wpilib/<year>/jdk` elsewhere).

## Using it

- **Drive:** in the Sim GUI, drag your controller onto `Joystick[0]` (driver) /
  `Joystick[1]` (copilot) and pick *Teleoperated*.
- **Watch:** AdvantageScope -> *File -> Connect to Simulator* -> 3D Field. Drag in
  `Sim/TruePose` (robot), `Localization/EstimatedPose` (ghost), `Sim/FieldBalls`
  and `Sim/Balls` (game pieces).
- **Test buttons:** `SmartDashboard/Sim/Commands/...` (shoot, shot sweep, intake,
  reset field). Sim settings live under `SmartDashboard/Sim/...`.
- **Tests:** `gradlew installFrcUserProgramTestDebugGoogleTestExe`, then run
  `build/install/frcUserProgramTest/debug/lib/frcUserProgramTest(.exe)` from this folder.
- **Shooting bots:** add `--gtest_also_run_disabled_tests --gtest_filter=ShotTuning*`.
  Results go to `build/shot_tuning/*.csv`.

## A new season

Most of the sim carries over every year. It is split into three parts:

| Part | Files | New season? |
|---|---|---|
| **Core** | `sim/*.cpp` (swerve, intake, shooter, ball flight, field physics, Limelights), `sim/hardware/` | keep |
| **Season** (the game) | `sim/season/SeasonYYYY.h` (numbers), `src/main/cpp/sim/season/SeasonYYYY.cpp` (field shapes, game-piece layout, goal) | new pair per game |
| **Robot** (the machine + its code) | `sim/robot/RobotConfig.h` (sizes, gear ratios, cameras), `sim/robot/RobotBinding.cpp` (which motor is which), `sim/robot/SimHook.cpp` | rewrite per robot |

**New game:**
1. Copy `Season2026.h` / `Season2026.cpp` to `SeasonYYYY`, change the numbers, field
   shapes, starting game pieces and goal. Change the `#if SIM_SEASON_YEAR == 2026` line.
2. In `sim/SimConstants.h`, set `SIM_SEASON_YEAR` and include the new season header.
3. Update WPILib (`build.gradle` GradleRIO version) and the vendor libraries in the robot code.

**New robot:**
1. `RobotConfig.h`: sizes, mass, gear ratios, mechanism limits, Limelight mounts.
2. `RobotBinding.cpp`: point each mechanism at the robot code's motor objects
   (`SparkMotor`, `CANcoderSensor`, `Pigeon2Gyro`, ...), and make the shoot / intake
   test-button commands from the robot's own commands.
3. In the robot code's `Robot.cpp`, keep the two sim lines (`g_simRobotContainer` and
   `SimulationInit`) so the sim can find the robot.

**Different hardware** (e.g. TalonFX motors): add an adapter class next to `SparkMotor`
in `sim/hardware/SimHardware.h` and use it in the binding. Nothing else changes.

**Different mechanisms** (no turret, an elevator, ...): the mechanism sims are separate
classes (`IntakeSim`, `ShooterSim`, `DrivetrainSim`); add or skip one in `RobotSim`.
