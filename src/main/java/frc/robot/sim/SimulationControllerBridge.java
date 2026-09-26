package frc.robot.sim;

import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/** Bridges gamepad data from the Python simulation supervisor into HALSim. */
public final class SimulationControllerBridge {
  private static final int JOYSTICK_PORT = 0;
  private static final int AXIS_COUNT = 6;
  private static final int BUTTON_COUNT = 10;
  private static final int STALE_CYCLES = 25;

  private final DoubleArraySubscriber axes;
  private final BooleanArraySubscriber buttons;
  private final IntegerSubscriber pov;
  private final BooleanSubscriber connected;
  private final StringSubscriber name;
  private final StringSubscriber mode;
  private final IntegerSubscriber heartbeat;

  private long previousHeartbeat = -1;
  private int staleCycles = STALE_CYCLES;

  public SimulationControllerBridge() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("SimSupervisor");
    axes = table.getDoubleArrayTopic("Joystick0/Axes").subscribe(new double[0]);
    buttons = table.getBooleanArrayTopic("Joystick0/Buttons").subscribe(new boolean[0]);
    pov = table.getIntegerTopic("Joystick0/POV").subscribe(-1);
    connected = table.getBooleanTopic("Joystick0/Connected").subscribe(false);
    name = table.getStringTopic("Joystick0/Name").subscribe("Simulation Controller");
    mode = table.getStringTopic("Mode").subscribe("disabled");
    heartbeat = table.getIntegerTopic("Heartbeat").subscribe(-1);

    DriverStationSim.setJoystickAxisCount(JOYSTICK_PORT, AXIS_COUNT);
    DriverStationSim.setJoystickButtonCount(JOYSTICK_PORT, BUTTON_COUNT);
    DriverStationSim.setJoystickPOVCount(JOYSTICK_PORT, 1);
    DriverStationSim.setJoystickIsXbox(JOYSTICK_PORT, true);
  }

  /** Copies the newest supervisor state into HALSim, clearing stale input after 0.5 seconds. */
  public void update() {
    long currentHeartbeat = heartbeat.get();
    if (currentHeartbeat != previousHeartbeat) {
      previousHeartbeat = currentHeartbeat;
      staleCycles = 0;
    } else {
      staleCycles++;
    }

    boolean isConnected = connected.get() && staleCycles < STALE_CYCLES;
    double[] currentAxes = isConnected ? axes.get() : new double[0];
    boolean[] currentButtons = isConnected ? buttons.get() : new boolean[0];

    for (int axis = 0; axis < AXIS_COUNT; axis++) {
      DriverStationSim.setJoystickAxis(
          JOYSTICK_PORT, axis, axis < currentAxes.length ? currentAxes[axis] : 0.0);
    }
    for (int button = 1; button <= BUTTON_COUNT; button++) {
      DriverStationSim.setJoystickButton(
          JOYSTICK_PORT,
          button,
          button <= currentButtons.length && currentButtons[button - 1]);
    }
    DriverStationSim.setJoystickPOV(JOYSTICK_PORT, 0, isConnected ? (int) pov.get() : -1);
    DriverStationSim.setJoystickName(
        JOYSTICK_PORT, isConnected ? name.get() : "Simulation Controller (disconnected)");

    String selectedMode = mode.get();
    boolean enabled = !selectedMode.equals("disabled") && staleCycles < STALE_CYCLES;
    DriverStationSim.setDsAttached(staleCycles < STALE_CYCLES);
    DriverStationSim.setAutonomous(selectedMode.equals("auto"));
    DriverStationSim.setTest(selectedMode.equals("test"));
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }
}
