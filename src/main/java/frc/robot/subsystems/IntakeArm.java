package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class IntakeArm extends SubsystemBase {

  // NOTE: these are measured with EMPIRICALLY measured angles for safety.
  // We couldn't figure out the zero offsets for the real robot so we just
  // used the actual angles output by the mechanism and noted the targets.
  private static final Angle STOWED_ANGLE = Degrees.of(190);
  private static final Angle RETRACTED_ANGLE = Degrees.of(250);
  private static final Angle EXTENDED_ANGLE = Degrees.of(295);
  private static final double KP = 5.0;
  private static final double KI = 0.0001;
  private static final double KD = 0;
  private static final double KS = 0;
  private static final double KG = 0;
  private static final double KV = 0;

  private SparkMax armMotor = new SparkMax(11, MotorType.kBrushless);

  private SmartMotorControllerConfig armSmartConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(KP, KI, KD)
      .withFeedforward(new ArmFeedforward(KS, KG, KV, 0))
      .withSimClosedLoopController(KP, KI, KD)
      .withSimFeedforward(new ArmFeedforward(KS, KG, KV, 0))
      .withTelemetry("IntakeArmMotor", ControlsConstants.YAMS_VERBOSITY)
      .withGearing(25 * (80 / 24))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withSoftLimit(STOWED_ANGLE, EXTENDED_ANGLE)
      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withUseExternalFeedbackEncoder(true)
      .withExternalEncoderZeroOffset(Degrees.zero())
      .withStartingPosition(Degrees.zero());

  private SmartMotorController armMotorController = YAMSUtil.safeGetSmartMotorController(
      armMotor,
      DCMotor.getNEO(1),
      armSmartConfig);

  private ArmConfig armMechanismConfig = new ArmConfig(armMotorController)
      // Hard limit is applied to the simulation.
      .withHardLimit(STOWED_ANGLE, EXTENDED_ANGLE)
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
  public IntakeArm() {
    setDefaultCommand(retractIntake());
    HardwareMonitor.registerHardware("intakeArmMotor", armMotor);
  }

  public Command allowCoast() {
    return new InstantCommand(() -> armMotorController.setIdleMode(MotorMode.COAST), this);
  }

  public Command enforceBrake() {
    return new InstantCommand(() -> armMotorController.setIdleMode(MotorMode.BRAKE), this);
  }

  public Command extendIntake() {
    return allowCoast().andThen(arm.setAngle(EXTENDED_ANGLE));
  }

  public Command retractIntake() {
    return enforceBrake().andThen(arm.setAngle(RETRACTED_ANGLE));
  }

  public Command stowIntake() {
    return enforceBrake().andThen(arm.setAngle(STOWED_ANGLE));
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
    SmartDashboard.putNumber("armDegrees", arm.getAngle().in(Degree));
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }

}