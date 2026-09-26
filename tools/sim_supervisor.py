#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "pygame-ce>=2.5,<3",
#   "pyntcore>=2026,<2027",
# ]
# ///
"""Restart a WPILib Java simulation whenever relevant project files change."""

from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


WATCH_DIRECTORIES = (
    Path("src/main/java"),
    Path("src/main/deploy"),
    Path("vendordeps"),
)
WATCH_FILES = (
    Path("build.gradle"),
    Path("settings.gradle"),
    Path("gradle.properties"),
    Path("gradle/wrapper/gradle-wrapper.properties"),
)
IGNORED_NAMES = {"BuildConstants.java", ".DS_Store"}


@dataclass(frozen=True)
class FileState:
    modified_ns: int
    size: int


Snapshot = dict[Path, FileState]


class GamepadPublisher:
    """Publish an SDL game controller using WPILib's Xbox controller layout."""

    # SDL_GameControllerAxis values. pygame-ce 2.5.8 accepts these stable SDL
    # enum values but does not export their symbolic names from _sdl2.controller.
    AXIS_LEFT_X = 0
    AXIS_LEFT_Y = 1
    AXIS_RIGHT_X = 2
    AXIS_RIGHT_Y = 3
    AXIS_TRIGGER_LEFT = 4
    AXIS_TRIGGER_RIGHT = 5

    # SDL_GameControllerButton values.
    BUTTON_A = 0
    BUTTON_B = 1
    BUTTON_X = 2
    BUTTON_Y = 3
    BUTTON_BACK = 4
    BUTTON_START = 6
    BUTTON_LEFT_STICK = 7
    BUTTON_RIGHT_STICK = 8
    BUTTON_LEFT_SHOULDER = 9
    BUTTON_RIGHT_SHOULDER = 10
    BUTTON_DPAD_UP = 11
    BUTTON_DPAD_DOWN = 12
    BUTTON_DPAD_LEFT = 13
    BUTTON_DPAD_RIGHT = 14

    AXIS_NAMES = ("Left X", "Left Y", "Left Trigger", "Right Trigger", "Right X", "Right Y")
    BUTTON_NAMES = ("A", "B", "X", "Y", "Left Bumper", "Right Bumper", "Back", "Start", "Left Stick", "Right Stick")

    def __init__(self, mode: str) -> None:
        os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
        import ntcore
        import pygame
        from pygame._sdl2 import controller

        self.pygame = pygame
        self.controller_module = controller
        pygame.init()
        controller.init()
        self.controller = None
        self.controller_name: str | None = None
        self.next_scan = 0.0
        self.heartbeat = 0
        self.selected_mode = mode
        self.previous_start = False
        self.previous_back = False
        self.logged_axes = [0.0] * 6
        self.axis_log_times = [0.0] * 6
        self.logged_buttons = [False] * 10
        self.logged_pov = -1

        instance = ntcore.NetworkTableInstance.getDefault()
        instance.startClient4("sim-supervisor")
        instance.setServer("127.0.0.1")
        table = instance.getTable("SimSupervisor")
        self.axes = table.getDoubleArrayTopic("Joystick0/Axes").publish()
        self.buttons = table.getBooleanArrayTopic("Joystick0/Buttons").publish()
        self.pov = table.getIntegerTopic("Joystick0/POV").publish()
        self.connected = table.getBooleanTopic("Joystick0/Connected").publish()
        self.name = table.getStringTopic("Joystick0/Name").publish()
        self.mode = table.getStringTopic("Mode").publish()
        self.heartbeat_topic = table.getIntegerTopic("Heartbeat").publish()
        self.mode.set(self.selected_mode)

    @staticmethod
    def _axis(value: int) -> float:
        return max(-1.0, min(1.0, value / 32767.0))

    @staticmethod
    def _trigger(value: int) -> float:
        return max(0.0, min(1.0, value / 32768.0))

    def _find_controller(self) -> None:
        now = time.monotonic()
        if now < self.next_scan:
            return
        self.next_scan = now + 1.0
        for index in range(self.controller_module.get_count()):
            if not self.controller_module.is_controller(index):
                continue
            try:
                self.controller = self.controller_module.Controller(index)
                joystick = self.controller.as_joystick()
                self.controller_name = joystick.get_name() or "SDL Game Controller"
                print(f"[gamepad] connected: {self.controller_name}", flush=True)
                return
            except self.pygame.error:
                if self.controller is not None:
                    self.controller.quit()
                    self.controller = None
                continue

    def _read_pov(self) -> int:
        c = self.controller
        up = c.get_button(self.BUTTON_DPAD_UP)
        right = c.get_button(self.BUTTON_DPAD_RIGHT)
        down = c.get_button(self.BUTTON_DPAD_DOWN)
        left = c.get_button(self.BUTTON_DPAD_LEFT)
        directions = {
            (True, False, False, False): 0,
            (True, True, False, False): 45,
            (False, True, False, False): 90,
            (False, True, True, False): 135,
            (False, False, True, False): 180,
            (False, False, True, True): 225,
            (False, False, False, True): 270,
            (True, False, False, True): 315,
        }
        return directions.get((up, right, down, left), -1)

    def _log_input(
        self, axes: list[float], buttons: list[bool], pov: int
    ) -> None:
        now = time.monotonic()
        for index, value in enumerate(axes):
            if (
                abs(value - self.logged_axes[index]) >= 0.05
                and now - self.axis_log_times[index] >= 0.1
            ):
                print(
                    f"[input] {self.AXIS_NAMES[index]}: {value:+.2f}",
                    flush=True,
                )
                self.logged_axes[index] = value
                self.axis_log_times[index] = now

        for index, pressed in enumerate(buttons):
            if pressed and not self.logged_buttons[index]:
                print(f"[input] {self.BUTTON_NAMES[index]} pressed", flush=True)
        self.logged_buttons = buttons.copy()

        if pov != self.logged_pov:
            label = "released" if pov == -1 else f"{pov}°"
            print(f"[input] D-pad: {label}", flush=True)
            self.logged_pov = pov

    def _reset_logged_input(self) -> None:
        self.logged_axes = [0.0] * 6
        self.logged_buttons = [False] * 10
        self.logged_pov = -1

    def update(self) -> None:
        self.pygame.event.pump()
        if self.controller is not None and not self.controller.attached():
            print(f"[gamepad] disconnected: {self.controller_name}", flush=True)
            self.controller.quit()
            self.controller = None
            self.controller_name = None

        if self.controller is None:
            self._find_controller()

        if self.controller is None:
            self.axes.set([0.0] * 6)
            self.buttons.set([False] * 10)
            self.pov.set(-1)
            self.connected.set(False)
            self.name.set("")
            self.previous_start = False
            self.previous_back = False
            self._reset_logged_input()
        else:
            c = self.controller
            start = c.get_button(self.BUTTON_START)
            back = c.get_button(self.BUTTON_BACK)
            if start and not self.previous_start:
                self.selected_mode = "teleop"
                print("[gamepad] Driver Station: TELEOP enabled", flush=True)
            elif back and not self.previous_back:
                self.selected_mode = "auto"
                print("[gamepad] Driver Station: AUTO enabled", flush=True)
            self.previous_start = start
            self.previous_back = back

            # WPILib Xbox axes: LX, LY, LT, RT, RX, RY.
            current_axes = [
                self._axis(c.get_axis(self.AXIS_LEFT_X)),
                self._axis(c.get_axis(self.AXIS_LEFT_Y)),
                self._trigger(c.get_axis(self.AXIS_TRIGGER_LEFT)),
                self._trigger(c.get_axis(self.AXIS_TRIGGER_RIGHT)),
                self._axis(c.get_axis(self.AXIS_RIGHT_X)),
                self._axis(c.get_axis(self.AXIS_RIGHT_Y)),
            ]
            self.axes.set(current_axes)
            # WPILib Xbox buttons: A, B, X, Y, LB, RB, Back, Start, LS, RS.
            # Back and Start are consumed as Driver Station controls.
            physical_buttons = [
                c.get_button(self.BUTTON_A),
                c.get_button(self.BUTTON_B),
                c.get_button(self.BUTTON_X),
                c.get_button(self.BUTTON_Y),
                c.get_button(self.BUTTON_LEFT_SHOULDER),
                c.get_button(self.BUTTON_RIGHT_SHOULDER),
                back,
                start,
                c.get_button(self.BUTTON_LEFT_STICK),
                c.get_button(self.BUTTON_RIGHT_STICK),
            ]
            forwarded_buttons = physical_buttons.copy()
            forwarded_buttons[6] = False
            forwarded_buttons[7] = False
            self.buttons.set(forwarded_buttons)
            current_pov = self._read_pov()
            self.pov.set(current_pov)
            self.connected.set(True)
            self.name.set(self.controller_name or "SDL Game Controller")
            self._log_input(current_axes, physical_buttons, current_pov)

        self.mode.set(self.selected_mode)
        self.heartbeat += 1
        self.heartbeat_topic.set(self.heartbeat)

    def close(self) -> None:
        self.connected.set(False)
        self.axes.set([0.0] * 6)
        self.buttons.set([False] * 10)
        self.pov.set(-1)
        if self.controller is not None:
            self.controller.quit()
        self.pygame.quit()


def project_year(root: Path) -> str | None:
    """Read the season year from the GradleRIO plugin version."""
    try:
        build_file = (root / "build.gradle").read_text()
    except OSError:
        return None
    match = re.search(r'GradleRIO["\']\s+version\s+["\'](\d{4})\.', build_file)
    return match.group(1) if match else None


def java_executable(java_home: Path) -> Path:
    return java_home / "bin" / ("java.exe" if os.name == "nt" else "java")


def find_java_home(root: Path, override: Path | None = None) -> Path | None:
    """Find a JDK, preferring the one installed with this WPILib season."""
    year = project_year(root)
    candidates: list[Path] = []
    if override is not None:
        candidates.append(override.expanduser())
    if configured := os.environ.get("JAVA_HOME"):
        candidates.append(Path(configured).expanduser())
    if year:
        candidates.extend(
            (
                Path.home() / "wpilib" / year / "jdk",
                Path("/Users/Shared/wpilib") / year / "jdk",
                Path("C:/Users/Public/wpilib") / year / "jdk",
            )
        )

    for candidate in candidates:
        candidate = candidate.resolve()
        if java_executable(candidate).is_file():
            return candidate

    # A system-installed macOS JDK may not have JAVA_HOME exported.
    java_home_tool = Path("/usr/libexec/java_home")
    if java_home_tool.is_file():
        result = subprocess.run(
            [str(java_home_tool)],
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0 and result.stdout.strip():
            candidate = Path(result.stdout.strip())
            if java_executable(candidate).is_file():
                return candidate
    return None


def snapshot(root: Path) -> Snapshot:
    """Return the state of all inputs that should restart simulation."""
    result: Snapshot = {}
    candidates: Iterable[Path] = (
        path
        for directory in WATCH_DIRECTORIES
        if (root / directory).is_dir()
        for path in (root / directory).rglob("*")
    )

    for path in candidates:
        if not path.is_file() or path.name in IGNORED_NAMES:
            continue
        try:
            stat = path.stat()
        except FileNotFoundError:
            continue
        result[path.relative_to(root)] = FileState(stat.st_mtime_ns, stat.st_size)

    for relative_path in WATCH_FILES:
        path = root / relative_path
        try:
            stat = path.stat()
        except FileNotFoundError:
            continue
        result[relative_path] = FileState(stat.st_mtime_ns, stat.st_size)

    return result


def changed_paths(before: Snapshot, after: Snapshot) -> list[Path]:
    """List created, deleted, and modified paths between two snapshots."""
    return sorted(
        path
        for path in before.keys() | after.keys()
        if before.get(path) != after.get(path)
    )


class Simulator:
    def __init__(
        self, root: Path, java_home: Path, extra_gradle_args: list[str]
    ) -> None:
        self.root = root
        self.java_home = java_home
        self.extra_gradle_args = extra_gradle_args
        self.process: subprocess.Popen[bytes] | None = None

    def start(self) -> None:
        wrapper = "gradlew.bat" if os.name == "nt" else "gradlew"
        command = [str(self.root / wrapper), "simulateJava", "--console=plain"]
        command.extend(self.extra_gradle_args)

        print(f"\n[sim] starting: {' '.join(command)}", flush=True)
        environment = os.environ.copy()
        environment["JAVA_HOME"] = str(self.java_home)
        environment["PATH"] = os.pathsep.join(
            (str(self.java_home / "bin"), environment.get("PATH", ""))
        )
        options: dict[str, object] = {"cwd": self.root, "env": environment}
        if os.name == "nt":
            options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            options["start_new_session"] = True
        self.process = subprocess.Popen(command, **options)  # type: ignore[arg-type]

    def stop(self, timeout: float = 5.0) -> None:
        process = self.process
        self.process = None
        if process is None or process.poll() is not None:
            return

        print("[sim] stopping current simulator...", flush=True)
        if os.name == "nt":
            process.send_signal(signal.CTRL_BREAK_EVENT)
        else:
            os.killpg(process.pid, signal.SIGINT)

        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass

        if os.name == "nt":
            subprocess.run(
                ["taskkill", "/PID", str(process.pid), "/T", "/F"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        else:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    def restart(self) -> None:
        self.stop()
        self.start()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Watch this robot project and restart HALSim after changes."
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
        help="robot project root (defaults to the parent of tools/)",
    )
    parser.add_argument(
        "--java-home",
        type=Path,
        help="JDK directory (defaults to the JDK bundled with this WPILib season)",
    )
    parser.add_argument(
        "--mode",
        choices=("disabled", "teleop", "auto", "test"),
        default="disabled",
        help="initial simulated Driver Station mode (default: disabled)",
    )
    parser.add_argument(
        "--debounce",
        type=float,
        default=0.35,
        help="seconds of quiet time before restarting (default: 0.35)",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.15,
        help="seconds between filesystem scans (default: 0.15)",
    )
    parser.add_argument(
        "gradle_args",
        nargs=argparse.REMAINDER,
        help="extra Gradle arguments after --, for example: -- --offline",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    wrapper = root / ("gradlew.bat" if os.name == "nt" else "gradlew")
    if not wrapper.is_file() or not (root / "build.gradle").is_file():
        print(f"error: {root} is not a WPILib Gradle project", file=sys.stderr)
        return 2
    if args.debounce < 0 or args.poll_interval <= 0:
        print("error: debounce must be nonnegative and poll interval positive", file=sys.stderr)
        return 2

    java_home = find_java_home(root, args.java_home)
    if java_home is None:
        year = project_year(root) or "YEAR"
        print(
            "error: no Java runtime found\n"
            f"Expected the WPILib JDK at ~/wpilib/{year}/jdk. "
            "Install WPILib or pass --java-home /path/to/jdk.",
            file=sys.stderr,
        )
        return 2

    gradle_args = args.gradle_args
    if gradle_args[:1] == ["--"]:
        gradle_args = gradle_args[1:]

    simulator = Simulator(root, java_home, gradle_args)
    gamepad = GamepadPublisher(args.mode)
    current = snapshot(root)
    pending: set[Path] = set()
    last_change = 0.0
    next_scan = 0.0

    print(f"[watch] project: {root}")
    print(f"[watch] Java: {java_home}")
    print("[watch] press Ctrl-C to stop")
    simulator.start()

    try:
        while True:
            time.sleep(0.02)
            gamepad.update()
            now = time.monotonic()
            if now >= next_scan:
                next_scan = now + args.poll_interval
                updated = snapshot(root)
                changes = changed_paths(current, updated)
                current = updated
                if changes:
                    pending.update(changes)
                    last_change = now

            if pending and now - last_change >= args.debounce:
                summary = ", ".join(str(path) for path in sorted(pending))
                print(f"\n[watch] changed: {summary}", flush=True)
                pending.clear()
                simulator.restart()
    except KeyboardInterrupt:
        print("\n[watch] shutting down...", flush=True)
    finally:
        gamepad.close()
        simulator.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
