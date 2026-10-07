package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/** Bridges dashboard driver-station and gamepad data into HALSim. */
public final class SimulationControllerBridge {
  private static final int JOYSTICK_COUNT = 2;
  private static final int AXIS_COUNT = 6;
  private static final int BUTTON_COUNT = 10;
  private static final int STALE_CYCLES = 25;

  private final JoystickState[] joysticks = new JoystickState[JOYSTICK_COUNT];
  private final StringSubscriber mode;
  private final StringSubscriber alliance;
  private final IntegerSubscriber heartbeat;
  private final BooleanPublisher available;

  private long previousHeartbeat = -1;
  private int staleCycles = STALE_CYCLES;

  public SimulationControllerBridge() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SimSupervisor");
    for (int port = 0; port < JOYSTICK_COUNT; port++) {
      joysticks[port] = new JoystickState(table, port);
      DriverStationSim.setJoystickAxisCount(port, AXIS_COUNT);
      DriverStationSim.setJoystickButtonCount(port, BUTTON_COUNT);
      DriverStationSim.setJoystickPOVCount(port, 1);
      DriverStationSim.setJoystickIsXbox(port, true);
    }
    mode = table.getStringTopic("Mode").subscribe("disabled");
    alliance = table.getStringTopic("Alliance").subscribe("blue");
    heartbeat = table.getIntegerTopic("Heartbeat").subscribe(-1);
    available = table.getBooleanTopic("Available").publish();
    available.set(true);
  }

  /**
   * Copies the newest supervisor state into HALSim, clearing stale input after
   * 0.5 seconds.
   */
  public void update() {
    long currentHeartbeat = heartbeat.get();
    if (currentHeartbeat != previousHeartbeat) {
      previousHeartbeat = currentHeartbeat;
      staleCycles = 0;
    } else {
      staleCycles++;
    }

    for (int port = 0; port < JOYSTICK_COUNT; port++) {
      JoystickState joystick = joysticks[port];
      boolean isConnected = joystick.connected.get() && staleCycles < STALE_CYCLES;
      double[] currentAxes = isConnected ? joystick.axes.get() : new double[0];
      boolean[] currentButtons = isConnected ? joystick.buttons.get() : new boolean[0];
      for (int axis = 0; axis < AXIS_COUNT; axis++) {
        DriverStationSim.setJoystickAxis(
            port, axis, axis < currentAxes.length ? currentAxes[axis] : 0.0);
      }
      for (int button = 1; button <= BUTTON_COUNT; button++) {
        DriverStationSim.setJoystickButton(
            port,
            button,
            button <= currentButtons.length && currentButtons[button - 1]);
      }
      DriverStationSim.setJoystickPOV(
          port, 0, isConnected ? (int) joystick.pov.get() : -1);
      DriverStationSim.setJoystickName(
          port,
          isConnected ? joystick.name.get() : "Simulation Controller (disconnected)");
    }

    String selectedMode = mode.get();
    DriverStationSim.setAllianceStationId(
        alliance.get().equals("red") ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    boolean enabled = !selectedMode.equals("disabled") && staleCycles < STALE_CYCLES;
    DriverStationSim.setDsAttached(staleCycles < STALE_CYCLES);
    DriverStationSim.setAutonomous(selectedMode.equals("auto"));
    DriverStationSim.setTest(selectedMode.equals("test"));
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  private static final class JoystickState {
    final DoubleArraySubscriber axes;
    final BooleanArraySubscriber buttons;
    final IntegerSubscriber pov;
    final BooleanSubscriber connected;
    final StringSubscriber name;

    JoystickState(NetworkTable table, int port) {
      String prefix = "Joystick" + port + "/";
      axes = table.getDoubleArrayTopic(prefix + "Axes").subscribe(new double[0]);
      buttons = table.getBooleanArrayTopic(prefix + "Buttons").subscribe(new boolean[0]);
      pov = table.getIntegerTopic(prefix + "POV").subscribe(-1);
      connected = table.getBooleanTopic(prefix + "Connected").subscribe(false);
      name = table.getStringTopic(prefix + "Name").subscribe("Simulation Controller " + port);
    }
  }
}
