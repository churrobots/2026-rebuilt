package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HardwareMonitor;
import frc.robot.util.TunableNumber;
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
  private static final Angle STOWED_ANGLE = Degrees.of(0.68 * 360);
  private static final Angle RETRACTED_ANGLE = Degrees.of(0.488 * 360);
  private static final Angle EXTENDED_ANGLE = Degrees.of(0.34 * 360);
  private static final double PULSING_PERIOD_SECONDS = 1.0;
  private static final double PULSING_SWEEP_ANGLE_DEGREES = 30.0;
  private static final Angle PULSING_ANGLE = EXTENDED_ANGLE.minus(Degrees.of(PULSING_SWEEP_ANGLE_DEGREES));
  private static final Angle DUTY_CYCLE_OFFSET = Degrees.of(180);
  private static final double KP = 4.5; // was 5.0
  private static final double KI = 0.3; // was 0.0001
  private static final double KD = 0;
  private static final double KS = 0;
  private static final double KG = 0.1; // was 0.0
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
      // TODO: soft limit prevents responding to commands at all - figure out why
      // .withSoftLimit(STOWED_ANGLE, EXTENDED_ANGLE)
      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withUseExternalFeedbackEncoder(true)
      .withExternalEncoderZeroOffset(DUTY_CYCLE_OFFSET)
      .withStartingPosition(Degrees.zero());

  private TunableNumber tunablePulsingAngle = new TunableNumber("INTAKE_ARM_PULSING_ANGLE",
      PULSING_ANGLE.in(Degrees));

  private SmartMotorController armMotorController = YAMSUtil.safeGetSmartMotorController(
      armMotor,
      DCMotor.getNEO(1),
      armSmartConfig);

  private ArmConfig armMechanismConfig = new ArmConfig(armMotorController)
      // Hard limit is applied to the simulation.
      .withHardLimit(STOWED_ANGLE, EXTENDED_ANGLE)
      // Length and mass of your arm for sim.
      .withLength(Feet.of(1))
      .withMass(Pounds.of(15))
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

  public Command extendIntake() {
    return arm.setAngle(EXTENDED_ANGLE);
  }

  public Command retractIntake() {
    return arm.setAngle(RETRACTED_ANGLE);
  }

  public Command stowIntake() {
    return arm.setAngle(STOWED_ANGLE);
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  public Command pulseArm() {
    return setAnglePulsed(() -> Degrees.of(tunablePulsingAngle.getLatest()), PULSING_PERIOD_SECONDS,
        PULSING_SWEEP_ANGLE_DEGREES);
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

  public Command setAnglePulsed(Supplier<Angle> angleSupplier, double periodInSeconds, double sweepAngleDegrees) {
    double startTime = Timer.getFPGATimestamp();

    Supplier<Angle> pulsedSupplier = () -> {
      double elapsed = Timer.getFPGATimestamp() - startTime;

      // Sine wave formula: sin(2π * frequency * time)
      double sineValue = Math.sin((2 * Math.PI / periodInSeconds) * elapsed);
      double angleDiff = (sweepAngleDegrees / 2) * sineValue;

      var actual = angleSupplier.get();
      var pulsed = actual.plus(Degrees.of(angleDiff));
      return pulsed;
    };
    return arm.setAngle(pulsedSupplier);
  }

}