# Simulation tools

## Java simulator supervisor

`SimSupervisor.java` keeps the latest robot code running in HALSim. It watches Java sources, deploy files, vendor dependencies, and Gradle configuration; save bursts are debounced into one clean restart. SimGUI remains disabled and AdvantageKit publishes NT4 for AdvantageScope and browser clients.

Run from the repository root with the WPILib JDK:

```bash
~/wpilib/2026/jdk/bin/java tools/SimSupervisor.java
```

An alternate project directory can be supplied as the first argument. Stop the supervisor and simulator with Ctrl-C.

The supervisor uses `--no-daemon` so the simulator remains in a process tree that can be terminated reliably during restarts. Gradle's normal incremental build cache is still used.

## Browser dashboard

The robot serves a zero-build dashboard from `src/main/deploy/dashboard` on port 5800.
Open `http://localhost:5800` in simulation or `http://10.TE.AM.2:5800` on a robot.
It connects directly to the NT4 server and publishes these topics under the `SimSupervisor` table:

On `localhost`, the dashboard installs as a PWA and keeps atomic, versioned offline snapshots.
Service workers require a secure context, so browsers disable PWA caching when the same dashboard is
opened from a roboRIO's plain-HTTP IP address; telemetry and controls still work there.

| Topic | NT type | Value |
| --- | --- | --- |
| `Available` | `boolean` | Published by the simulation bridge; reveals the faux DS |
| `Joystick0/Axes` | `double[]` | WPILib Xbox order: LX, LY, LT, RT, RX, RY |
| `Joystick0/Buttons` | `boolean[]` | A, B, X, Y, LB, RB, Back, Start, LS, RS |
| `Joystick0/POV` | `int` | Degrees clockwise from up, or `-1` |
| `Joystick0/Connected` | `boolean` | Gamepad connection state |
| `Joystick0/Name` | `string` | Display name |
| `Mode` | `string` | `disabled`, `teleop`, `auto`, or `test` |
| `Alliance` | `string` | `blue` or `red` |
| `Heartbeat` | `int` | Increment continuously, ideally at 50 Hz |

The robot-side `SimulationControllerBridge` copies these values into `DriverStationSim`. If the heartbeat stops for approximately half a second, it clears all controls and disables the simulated robot.
