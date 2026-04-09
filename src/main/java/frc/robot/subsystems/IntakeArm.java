package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.HardwareMonitor;
import frc.robot.util.SemiAutoHelper;
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
  private static final Angle STOWED_ANGLE = Degrees.of(0.66 * 360);
  private static final Angle RETRACTED_ANGLE = Degrees.of(0.50 * 360);
  private static final Angle EXTENDED_ANGLE = Degrees.of(0.34 * 360);
  private static final Angle DEPOT_PREP_ANGLE = EXTENDED_ANGLE.minus(Degrees.of(25)); // TODO: measure this

  // Pulsing constants
  private static final double PULSING_PERIOD_SECONDS = 1;
  private static final Angle PULSING_ANGLE_START = Degrees.of(0.40 * 360);
  private static final Angle PULSING_ANGLE_END = Degrees.of(0.63 * 360);
  private static final double PULSING_CUTOFF_PERCENT = 0.1;

  private static final Angle DUTY_CYCLE_OFFSET = Degrees.of(180);
  private static final double KP = 4.5; // was 5.0
  private static final double KI = 0.0001; // was 0.0001
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
      .withSoftLimit(EXTENDED_ANGLE, STOWED_ANGLE)
      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withUseExternalFeedbackEncoder(true)
      .withExternalEncoderZeroOffset(DUTY_CYCLE_OFFSET)
      .withStartingPosition(Degrees.zero());

  private TunableNumber tunablePulsingAngleStart = new TunableNumber(
      "PULSING_ANGLE_START",
      PULSING_ANGLE_START.in(Degrees));
  private TunableNumber tunablePulsingAngleEnd = new TunableNumber(
      "PULSING_ANGLE_END",
      PULSING_ANGLE_END.in(Degrees));
  private TunableNumber tunablePulsingAnglePeriodInSeconds = new TunableNumber(
      "PULSING_PERIOD_SECONDS",
      PULSING_PERIOD_SECONDS);

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
  private Drive drive;

  /**
   * Creates a new ExampleSubsystem.
   */
  public IntakeArm(Drive drive) {
    this.drive = drive;
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
    double startTime = Timer.getFPGATimestamp();

    Supplier<Angle> pulsedSupplier = () -> {
      double elapsed = Timer.getFPGATimestamp() - startTime;

      // Get latest values.
      double periodInSeconds = tunablePulsingAnglePeriodInSeconds.getLatest();
      double startAngleDegrees = tunablePulsingAngleStart.getLatest();
      double endAngleDegrees = tunablePulsingAngleEnd.getLatest();
      double lowestAngleDegrees = Math.min(startAngleDegrees, endAngleDegrees);
      double highestAngleDegrees = Math.max(startAngleDegrees, endAngleDegrees);
      double sweepAngleDegrees = highestAngleDegrees - lowestAngleDegrees;

      // Sine wave formula: sin(2π * frequency * time)
      // We add a bit of excess so we can "cut off" the sine
      // wave and get some flat spots at top and bottom, to
      // prevent the gears from grinding from quick swiches
      // between forward and backward motion.
      double sineValue = Math.sin((2 * Math.PI / periodInSeconds) * elapsed);
      double angleDiffDegrees = (sweepAngleDegrees / 2) * (sineValue + 1);
      double angleDiffDegreesExpandedToAchieveCutoff = (1 + PULSING_CUTOFF_PERCENT) * angleDiffDegrees;
      double pulsedAngleDegrees = lowestAngleDegrees + angleDiffDegreesExpandedToAchieveCutoff;
      double cutoffPulsedAngleDegrees = MathUtil.clamp(pulsedAngleDegrees, lowestAngleDegrees, highestAngleDegrees);
      return Degrees.of(cutoffPulsedAngleDegrees);
    };
    return arm.setAngle(pulsedSupplier);
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();

    // Do the autonomous state.
    if (DriverStation.isAutonomous()) {
      if (autonomousState == AutonomousState.INTAKE) {
        boolean needsSafety = SemiAutoHelper.isInTrenchBumpZone(drive);
        if (needsSafety) {
          armMotorController.setPosition(RETRACTED_ANGLE);
        } else {
          armMotorController.setPosition(EXTENDED_ANGLE);
        }
      } else if (autonomousState == AutonomousState.CHILLOUT) {
        armMotorController.setPosition(RETRACTED_ANGLE);
      } else if (autonomousState == AutonomousState.DEPOT_PREP) {
        armMotorController.setPosition(DEPOT_PREP_ANGLE);
      }
    }

  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }

  public static enum AutonomousState {
    OFF,
    CHILLOUT,
    INTAKE,
    DEPOT_PREP
  }

  private AutonomousState autonomousState = AutonomousState.OFF;

  public Command setDesiredAutonomousState(AutonomousState desiredState) {
    return run(() -> {
      autonomousState = desiredState;
    }).withTimeout(0);
  }

}