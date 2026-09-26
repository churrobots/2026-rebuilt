# Simulation supervisor

`sim_supervisor.py` runs the WPILib Java simulator without SimGUI and restarts it after robot code, deploy files, vendor dependencies, or Gradle configuration changes.

From the repository root:

```bash
uv run --script tools/sim_supervisor.py
```

The script contains its own uv metadata and requires Python 3.10 or newer. It has no third-party dependencies. On macOS and Linux it can also be launched directly:

```bash
./tools/sim_supervisor.py
```

The supervisor automatically uses the JDK installed at `~/wpilib/2026/jdk`. A custom installation can be selected explicitly:

```bash
uv run --script tools/sim_supervisor.py --java-home /path/to/jdk
```

Connect AdvantageScope to `localhost`. Stop the supervisor and its simulator with Ctrl-C.

The first SDL-compatible game controller is mapped to WPILib Xbox controller port 0. Simulation starts disabled. Press **Start** to enable teleop or **Select/Back** to enable autonomous; the selected mode persists after the button is released. A different initial Driver Station mode can be selected with:

```bash
uv run --script tools/sim_supervisor.py --mode disabled
uv run --script tools/sim_supervisor.py --mode auto
uv run --script tools/sim_supervisor.py --mode test
```

The gamepad can be connected or disconnected while the supervisor is running. A heartbeat clears the simulated controls and disables the robot if the Python input process stops responding.

The console logs button presses, D-pad changes, and axis movement. Axis logs use a 0.05 change threshold and are limited to ten updates per second per axis to suppress stick noise.

Extra Gradle arguments can be passed after `--`:

```bash
uv run --script tools/sim_supervisor.py -- --offline
```

The watcher uses polling deliberately, so it has no third-party Python dependencies and behaves consistently on macOS, Windows, Linux, network drives, and mounted workspaces. Its defaults can be adjusted with `--debounce` and `--poll-interval`; run with `--help` for details.

SimGUI is disabled by default in `build.gradle`. The simulated Driver Station extension remains enabled.
