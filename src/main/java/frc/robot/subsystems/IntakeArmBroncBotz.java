package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HardwareMonitor;
import frc.robot.util.YAMSUtil;

import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class IntakeArmBroncBotz extends SubsystemBase {

  private SparkMax armMotor = new SparkMax(11, MotorType.kBrushless);

  private SmartMotorControllerConfig armSmartConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(2.5, 0, 0.5)
      .withSimClosedLoopController(10, 0, 0)
      .withFeedforward(new ArmFeedforward(0.15, 0, 50, 0))
      .withSimFeedforward(new ArmFeedforward(0.25, 0, 0.25))
      .withTelemetry("IntakeArmMotor", ControlsConstants.YAMS_VERBOSITY)
      .withGearing(25 * (80 / 24))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withStartingPosition(Degrees.zero())
      .withSoftLimit(Degrees.of(0), Degrees.of(65))
      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withExternalEncoderZeroOffset(Degrees.zero())
      .withUseExternalFeedbackEncoder(true);

  private SmartMotorController armMotorController = YAMSUtil.safeGetSmartMotorController(
      armMotor,
      DCMotor.getNEO(1),
      armSmartConfig);

  private ArmConfig armMechanismConfig = new ArmConfig(armMotorController)
      // Hard limit is applied to the simulation.
      .withHardLimit(Degrees.of(-5), Degrees.of(60))
      // Length and mass of your arm for sim.
      .withLength(Feet.of(1))
      .withMass(Pounds.of(5))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("IntakeArm", ControlsConstants.YAMS_VERBOSITY);

  // Arm Mechanism
  private Arm arm = new Arm(armMechanismConfig);

  /**
   * Creates a new ExampleSubsystem.
   */
  public IntakeArmBroncBotz() {
    setDefaultCommand(retractIntake());
    HardwareMonitor.registerHardware("intakeArmMotor", armMotor);
  }

  public Command allowCoast() {
    return new InstantCommand(() -> armMotorController.setIdleMode(MotorMode.COAST));
  }

  public Command enforceBrake() {
    return new InstantCommand(() -> armMotorController.setIdleMode(MotorMode.BRAKE));
  }

  public Command extendIntake() {
    return Commands.parallel(
        allowCoast(),
        arm.setAngle(Degrees.of(0)));
  }

  public Command retractIntake() {
    return Commands.parallel(
        enforceBrake(),
        arm.setAngle(Degrees.of(-15)));
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }

}