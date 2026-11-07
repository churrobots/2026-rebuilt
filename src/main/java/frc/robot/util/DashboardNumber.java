package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * A number the dashboard can change while the robot runs (like the shot trim).
 *
 * <p>The robot only LISTENS to the NetworkTables topic; the dashboard owns it. (With
 * AdvantageKit's LoggedNetworkNumber, the robot also publishes a starting value, and the
 * dashboard's changes were being ignored.) Each read is logged as an AdvantageKit input, so log
 * replay sees the same values the robot used. Until the dashboard sends something, it's the
 * default from the code.
 */
public class DashboardNumber {
  @AutoLog
  public static class DashboardNumberInputs {
    public double value = 0.0;
  }

  private final DoubleSubscriber subscriber;
  private final DashboardNumberInputsAutoLogged inputs = new DashboardNumberInputsAutoLogged();
  private final String logKey;
  private final double defaultValue;

  /**
   * @param topic the NetworkTables topic the dashboard publishes to, like "/Tuning/ShotTrimRpm"
   * @param logKey where AdvantageKit logs it, like "Tuning/ShotTrimRpm"
   * @param defaultValue the value to use until the dashboard sends one
   */
  public DashboardNumber(String topic, String logKey, double defaultValue) {
    this.subscriber =
        NetworkTableInstance.getDefault().getDoubleTopic(topic).subscribe(Double.NaN);
    this.logKey = logKey;
    this.defaultValue = defaultValue;
    inputs.value = defaultValue;
  }

  /** The current value: the dashboard's, or the default if it hasn't sent one. */
  public double get() {
    double fromDashboard = subscriber.get();
    inputs.value = Double.isNaN(fromDashboard) ? defaultValue : fromDashboard;
    Logger.processInputs(logKey, inputs);
    return inputs.value;
  }
}
