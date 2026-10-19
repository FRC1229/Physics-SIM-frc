# Running the sim in Docker

Docker runs a program inside a **container**, a small, separate Linux system
with exactly the right tools already installed. Here it means you can build and
run the sim **without installing WPILib**, and everyone gets the same setup.

## Do I need this?

| Your computer | What to use |
|---|---|
| Windows or macOS | The normal setup in the [main README](../README.md). (Docker on Windows is being tested; see [below](#windows-help-wanted).) |
| Linux, Ubuntu 24.04 or newer | Either works. The normal setup is simpler. |
| Linux, Ubuntu 22.04 or older | **Use Docker.** The normal build fails at the very end with `undefined reference to ...@GLIBCXX_3.4.32`. |

Why the error? The 2026 vendor libraries (for example PhotonLib) were built with
a newer C++ compiler than older Linux versions come with. The container uses
Ubuntu 24.04, so they work there.

## One-time setup (Linux)

1. **Install Docker Engine:** follow <https://docs.docker.com/engine/install/ubuntu/>.
2. **Let your user run Docker** without `sudo`, then log out and back in:
   ```bash
   sudo usermod -aG docker $USER
   ```
3. **NVIDIA graphics card?** Install the
   [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
   so the 3D views can use the GPU. Intel and AMD graphics work without anything extra.
4. **Put the robot code next to this folder**, the same as the normal setup:
   ```
   <any folder>/
     Rebuilt/            <- the robot code
     Physics-SIM-frc/    <- this sim
   ```

You don't need WPILib, Java or a C++ compiler on your computer. The container has them.

## Everyday use

Run these from the `Physics-SIM-frc` folder:

| Command | What it does |
|---|---|
| `docker/sim.sh` | Builds the robot code + sim and opens the Sim GUI |
| `docker/sim.sh scope` | Opens AdvantageScope (the 3D field). Then *File -> Connect to Simulator* |
| `docker/sim.sh build` | Builds everything and runs the automated tests |
| `docker/sim.sh check` | Only runs the tests |
| `docker/sim.sh shell` | Opens a terminal inside the container (for poking around) |

Run the sim and AdvantageScope in **two separate terminals** so both stay open.
Driving and the 3D field work the same as in the [main README](../README.md#using-it).

The **first run takes a while** and needs internet: it builds the container
(about 1.5 GB) and downloads the WPILib libraries (about 3 GB). Later runs reuse
all of that and start much faster.

## Good to know

- **Your computer stays usable while it builds.** The container uses half your
  CPU cores at a lower priority than your other apps. For a faster build, give
  it more: `SIM_CPUS=8 docker/sim.sh build`.
- **Only one sim at a time.** A second copy couldn't talk to AdvantageScope, so
  `sim.sh` refuses to start one. Close the old Sim GUI window first.
- **Robot code somewhere else?** Point to it:
  `ROBOT_PROJECT_DIR=~/code/MyRobot docker/sim.sh`.
- **Downloaded libraries** are kept in `~/.gradle-frc-docker`, and
  AdvantageScope's saved layout in `~/.frc-sim-docker-home`. Deleting either just
  means the next run downloads or sets things up again.

## Troubleshooting

| Problem | Fix |
|---|---|
| `permission denied ... docker.sock` | Step 2 of the setup, then log out and back in. |
| No window appears / `cannot open display` | You need a desktop session (X11). Run `echo $DISPLAY`; it should print something like `:0`. |
| AdvantageScope's 3D field is black or blank | NVIDIA: install the Container Toolkit (setup step 3). |
| `Already running (container 'frc1229-sim')` | A sim is already open. Close its window, or run `docker stop frc1229-sim`. |
| `Robot project not found at ...` | The `Rebuilt` folder isn't next to this one. Move it, or set `ROBOT_PROJECT_DIR`. |
| Robot doesn't move | Click *Teleoperated* in the Sim GUI's **Robot State** panel. |

## How it works

- [`Dockerfile`](Dockerfile) describes the container: Ubuntu 24.04 + Java 17 +
  the C++ compiler + AdvantageScope.
- [`sim.sh`](sim.sh) builds that container if needed, then runs Gradle (or
  AdvantageScope) inside it. The project folders are shared with the container,
  so build output lands in the normal `build/` folder. The container's windows
  show up on your desktop, and your controllers and graphics card are passed in.

## Windows: help wanted

This has **only been tested on Linux** so far. On Windows, the normal setup
(WPILib + Visual Studio Build Tools) is the way to go for now. If you'd like to
help get Docker working on Windows (one setup for everyone, and the base for
running tests automatically on GitHub), follow the testing checklist in
[pull request #1](https://github.com/FRC1229/Physics-SIM-frc/pull/1) and comment
with what happened.
