// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.HardwareMonitor;
import frc.robot.util.SemiAutoHelper;
import frc.robot.util.YAMSUtil;
import frc.robot.subsystems.sotm.ShotCalculator;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Churret extends SubsystemBase {

  // Tolerance for angle accuracy (degrees)
  private static final double ANGLE_TOLERANCE_DEGREES = 2.0;

  private final TalonFX churretMotor = new TalonFX(48);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      // Configure Motor and Mechanism properties
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(100))) // 100:1 gear ratio built into Falcon
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      // Setup Telemetry
      .withTelemetry("ChurretMotor", TelemetryVerbosity.HIGH)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  private final SmartMotorController churretController = YAMSUtil.safeGetSmartMotorController(
      churretMotor,
      DCMotor.getFalcon500(1),
      motorConfig);

  private final PivotConfig pivotConfig = new PivotConfig(churretController)
      .withStartingPosition(Degrees.of(90)) // Starting position (center of half rotation range)
      .withHardLimit(Degrees.of(0), Degrees.of(180)) // Half rotation: 180° total travel
      .withTelemetry("Churret", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(Meters.of(0.25), Pounds.of(4)); // MOI Calculation

  // Pivot Mechanism
  private final Pivot pivot = new Pivot(pivotConfig);

  // Simulation visualization
  private final Mechanism2d mechViz = new Mechanism2d(3, 3);
  private final MechanismRoot2d mechRoot = mechViz.getRoot("ChurretPivot", 1.5, 0.5);
  private final MechanismLigament2d turretArm = mechRoot.append(
      new MechanismLigament2d("TurretArm", 1.0, 90, 6, new Color8Bit(Color.kOrange)));
  private final MechanismLigament2d targetLine = mechRoot.append(
      new MechanismLigament2d("TargetLine", 0.8, 90, 2, new Color8Bit(Color.kGreen)));

  /**
   * Constructor - initializes churret with hardware monitoring.
   * Note: Default command is set in RobotContainer to allow access to Drive subsystem.
   */
  public Churret() {
    HardwareMonitor.registerHardware("churretMotor", churretMotor);

    // Publish visualization to SmartDashboard
    SmartDashboard.putData("Churret Mechanism", mechViz);
  }

  /**
   * Command to set the turret to a specific angle.
   * @param angle Target angle for the turret
   * @return Command that sets the turret angle
   */
  public Command setAngle(Angle angle) {
    targetAngle = angle;
    return pivot.setAngle(angle);
  }

  /**
   * Command to set the turret angle using a supplier (for dynamic aiming).
   * @param angleSupplier Supplier that provides the target angle
   * @return Command that continuously updates the turret angle
   */
  public Command setAngle(Supplier<Angle> angleSupplier) {
    return pivot.setAngle(angleSupplier);
  }

  /**
   * Command to aim the turret at the hub based on robot pose.
   * Accounts for physical shooter offset from robot center.
   * Uses SemiAutoHelper to calculate the angle to the hub.
   * @param drive Drive subsystem reference for calculations
   * @return Command that continuously aims the turret at the hub
   */
  public Command aimAtHub(Drive drive) {
    return setAngle(() -> {
      Pose2d robotPose = drive.getPose();

      // Get hub position and calculate angle from robot to hub
      Rotation2d angleToHub = SemiAutoHelper.getAngleToHub(drive);

      // Calculate turret angle relative to robot (field angle - robot angle)
      Rotation2d turretAngle = angleToHub.minus(robotPose.getRotation());

      // Clamp to valid range and normalize
      double clampedAngle = MathUtil.clamp(
          MathUtil.inputModulus(turretAngle.getDegrees(), 0, 360),
          0,
          180
      );

      Angle angle = Degrees.of(clampedAngle);
      targetAngle = angle;
      return angle;
    });
  }

  /**
   * Command to aim the turret at the alliance zone (for passing notes).
   * Points toward your alliance side: 0° for red alliance, 180° for blue alliance.
   * @param drive Drive subsystem reference for calculations
   * @return Command that aims the turret toward alliance
   */
  public Command aimAtAlliance(Drive drive) {
    return setAngle(() -> {
      Pose2d robotPose = drive.getPose();
      Rotation2d angleToAlliance = SemiAutoHelper.getAngleToAlliance();
      // Calculate turret angle relative to robot (field angle - robot angle)
      Rotation2d turretAngle = angleToAlliance.minus(robotPose.getRotation());

      // Clamp to valid range and normalize
      double clampedAngle = MathUtil.clamp(
          MathUtil.inputModulus(turretAngle.getDegrees(), 0, 360),
          0,
          180
      );

      Angle angle = Degrees.of(clampedAngle);
      targetAngle = angle;
      return angle;
    });
  }

  /**
   * Command for full auto-aim that intelligently switches targets.
   * - In neutral zone: aims at alliance (for passing)
   * - Outside neutral zone: aims at hub (for scoring)
   * Uses SemiAutoHelper.getFullAutoDriveAngle() logic.
   * @param drive Drive subsystem reference for calculations
   * @return Command that intelligently aims based on field position
   */
  public Command fullAutoAim(Drive drive) {
    return setAngle(() -> {
      Pose2d robotPose = drive.getPose();
      Rotation2d fieldTargetAngle = SemiAutoHelper.getFullAutoDriveAngle(drive);
      // Calculate turret angle relative to robot (field angle - robot angle)
      Rotation2d turretAngle = fieldTargetAngle.minus(robotPose.getRotation());

      // Clamp to valid range and normalize
      double clampedAngle = MathUtil.clamp(
          MathUtil.inputModulus(turretAngle.getDegrees(), 0, 360),
          0,
          180
      );

      Angle angle = Degrees.of(clampedAngle);
      targetAngle = angle;
      return angle;
    });
  }

  /**
   * Shoot-on-the-move (SOTM) auto-aim command with velocity compensation.
   * Accounts for robot movement, drag, and latency to accurately aim while driving.
   * Falls back to holding current position if SOTM solution is invalid or low confidence.
   * @param semiAutoHelper SemiAutoHelper instance for SOTM calculations
   * @param drive Drive subsystem reference for pose and velocity
   * @return Command that aims with velocity compensation
   */
  public Command aimWithSOTM(SemiAutoHelper semiAutoHelper, Drive drive) {
    return setAngle(() -> {
      // Calculate the SOTM firing solution
      ShotCalculator.LaunchParameters shot = semiAutoHelper.calculateSOTMShot();

      // Check if the solution is valid and confidence is acceptable
      if (!shot.isValid() || shot.confidence() < 50) {
        // Fall back to holding current angle if SOTM fails
        // This prevents erratic movement when the solver can't find a good solution
        return getCurrentAngle();
      }

      // Use the SOTM-calculated drive angle and convert to turret angle
      Pose2d robotPose = drive.getPose();
      Rotation2d turretAngle = shot.driveAngle().minus(robotPose.getRotation());

      // Clamp to valid turret range (0-180 degrees)
      double clampedAngle = MathUtil.clamp(
          MathUtil.inputModulus(turretAngle.getDegrees(), 0, 360),
          0,
          180
      );

      Angle angle = Degrees.of(clampedAngle);
      targetAngle = angle;

      // Publish SOTM telemetry for debugging
      SmartDashboard.putNumber("Churret/SOTM/Confidence", shot.confidence());
      SmartDashboard.putNumber("Churret/SOTM/RPM", shot.rpm());
      SmartDashboard.putNumber("Churret/SOTM/TOF", shot.timeOfFlightSec());
      SmartDashboard.putNumber("Churret/SOTM/Iterations", shot.iterationsUsed());
      SmartDashboard.putBoolean("Churret/SOTM/Valid", shot.isValid());

      return angle;
    });
  }

  /**
   * Command to aim the turret at a specific target position on the field.
   * @param target Target position on the field (Translation2d)
   * @param drive Drive subsystem for getting robot pose
   * @return Command that aims the turret at the target
   */
  public Command aimAtTarget(Translation2d target, Drive drive) {
    return setAngle(() -> {
      Pose2d robotPose = drive.getPose();
      // Calculate angle from robot to target
      Translation2d robotToTarget = target.minus(robotPose.getTranslation());
      Rotation2d angleToTarget = new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
      // Calculate turret angle relative to robot
      Rotation2d turretAngle = angleToTarget.minus(robotPose.getRotation());
      Angle angle = Degrees.of(turretAngle.getDegrees());
      targetAngle = angle;
      return angle;
    });
  }

  /**
   * Command to reset the turret to zero position (forward-facing relative to robot).
   * @return Command that resets turret to zero
   */
  public Command resetToZero() {
    return setAngle(Degrees.of(0));
  }

  /**
   * Gets the current angle of the turret.
   * @return Current turret angle
   */
  public Angle getCurrentAngle() {
    return pivot.getAngle();
  }

  private Angle targetAngle = Degrees.of(90); // Track target angle

  /**
   * Checks if the turret is within tolerance of its target angle.
   * @return true if turret is at target, false otherwise
   */
  public boolean isAtTarget() {
    double error = Math.abs(getCurrentAngle().in(Degrees) - targetAngle.in(Degrees));
    return error < ANGLE_TOLERANCE_DEGREES;
  }

  /**
   * Checks if the turret is ready to shoot (at target).
   * @return true if ready to shoot, false otherwise
   */
  public boolean isReadyToShoot() {
    return isAtTarget();
  }

  /**
   * Checks if the turret can physically reach a target angle.
   * @param targetAngle Angle to check (in degrees)
   * @return true if angle is within 0-180° range
   */
  public boolean canReachTarget(double targetAngle) {
    double normalizedAngle = MathUtil.inputModulus(targetAngle, 0, 360);
    return normalizedAngle >= 0 && normalizedAngle <= 180;
  }

  /**
   * Command to hold the turret at its current position.
   * @return Command that holds current position
   */
  public Command holdPosition() {
    return run(() -> {
      // Continuously set to current angle to hold position
      Angle currentAngle = getCurrentAngle();
      pivot.setAngle(currentAngle);
    }).withName("HoldChurretPosition");
  }

  /**
   * Emergency stop command - holds turret at current position.
   * @return Command that stops the turret
   */
  public Command emergencyStop() {
    return runOnce(() -> {
      targetAngle = getCurrentAngle();
    }).andThen(holdPosition()).withName("EmergencyStopChurret");
  }

  @Override
  public void periodic() {
    pivot.updateTelemetry();

    // Additional telemetry for debugging and driver feedback
    SmartDashboard.putBoolean("Churret/AtTarget", isAtTarget());
    SmartDashboard.putBoolean("Churret/ReadyToShoot", isReadyToShoot());
    SmartDashboard.putNumber("Churret/CurrentAngle", getCurrentAngle().in(Degrees));
    SmartDashboard.putNumber("Churret/TargetAngle", targetAngle.in(Degrees));
    SmartDashboard.putNumber("Churret/AngleError",
        getCurrentAngle().in(Degrees) - targetAngle.in(Degrees));

    // Mode indicator - shows which command is active
    String activeMode = "None";
    if (getCurrentCommand() != null) {
      activeMode = getCurrentCommand().getName();
    }
    SmartDashboard.putString("Churret/ActiveMode", activeMode);

    // Update visualization
    updateVisualization();
  }

  @Override
  public void simulationPeriodic() {
    pivot.simIterate();
  }

  /**
   * Updates the Mechanism2d visualization for the turret.
   */
  private void updateVisualization() {
    double currentAngleDeg = getCurrentAngle().in(Degrees);
    double targetAngleDeg = targetAngle.in(Degrees);

    // Update turret arm position (current angle)
    turretArm.setAngle(currentAngleDeg);

    // Update target line position
    targetLine.setAngle(targetAngleDeg);

    // Color code based on readiness
    if (isReadyToShoot()) {
      turretArm.setColor(new Color8Bit(Color.kGreen)); // Ready to shoot
    } else if (isAtTarget()) {
      turretArm.setColor(new Color8Bit(Color.kYellow)); // At target but not quite ready
    } else {
      turretArm.setColor(new Color8Bit(Color.kOrange)); // Still moving
    }
  }



}
