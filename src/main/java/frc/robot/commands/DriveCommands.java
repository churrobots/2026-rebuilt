// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AimHelper;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final double ALLOWABLE_ERROR_MARGIN_FOR_AIMING = 0.06;

  private DriveCommands() {
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Resets which way is "forward" to the way the robot is facing right now. Point
   * the robot
   * straight away from your driver station, then press the button. Keeps the
   * robot's x and y.
   */
  public static Command resetHeading(Drive drive) {
    return Commands.runOnce(
        () -> {
          // The field's 0° always points from the blue wall toward the red wall, so
          // "away from me" is 0° on blue and 180° on red.
          boolean isRed = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          double degrees = isRed ? 180.0 : 0.0;
          Pose2d pose = drive.getPose();
          drive.setPose(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(degrees)));
        },
        drive);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for
   * angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target,
   * or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<Boolean> xLockRequested) {

    // Create PID controller
    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP,
        0.0,
        ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Calculate angular speed
          Rotation2d targetHeading = rotationSupplier.get();
          double omega = angleController.calculate(
              drive.getRotation().getRadians(), targetHeading.getRadians());

          // Locked = the robot is really facing the target (not just following the profile).
          double headingError = targetHeading.minus(drive.getRotation()).getRadians();
          Logger.recordOutput("Aim/TargetHeadingDeg", targetHeading.getDegrees());
          Logger.recordOutput("Aim/ActualHeadingDeg", drive.getRotation().getDegrees());
          boolean locked = Math.abs(headingError) < ALLOWABLE_ERROR_MARGIN_FOR_AIMING;
          Logger.recordOutput("Aim/Locked", locked);
          drive.setAimLocked(locked);
          // For the dashboard's Aim card: how far off we are (+ = target is to the left,
          // counter-clockwise) and how close counts as locked.
          Logger.recordOutput("Aim/ErrorDeg", Math.toDegrees(headingError));
          Logger.recordOutput(
              "Aim/LockToleranceDeg", Math.toDegrees(ALLOWABLE_ERROR_MARGIN_FOR_AIMING));

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega);
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

          // Handle x-lock in a smart way, so that the robot doesn't
          // undershoot its goal angle before x-locking (resulting
          // in inaccurate aiming for example). Only xlock when we're
          // fully aimed. The driver can always break out: pushing the left stick
          // (past the dead zone) skips the x-lock, so a defender can't pin us.
          boolean wantsXLock = xLockRequested.get() == true;
          double error = angleController.getPositionError();
          boolean closeEnough = Math.abs(error) < ALLOWABLE_ERROR_MARGIN_FOR_AIMING;
          boolean driverMoving = linearVelocity.getNorm() > 0.0; // 0 inside the dead zone
          if (wantsXLock && closeEnough && !driverMoving) {
            Logger.recordOutput("Aim/XLocked", true);
            drive.stopWithX();
          } else {
            Logger.recordOutput("Aim/XLocked", false);
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds,
                    isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()));
          }
        },
        drive)

        // Reset the controller and dashboard state when angle drive begins.
        .beforeStarting(
            () -> {
              angleController.reset(drive.getRotation().getRadians());
              Logger.recordOutput("Aim/Locked", false);
              drive.setAimLocked(false);
            })

        // Commands may be cancelled before another drive command gets a chance to
        // publish.
        .finallyDo(
            () -> {
              Logger.recordOutput("Aim/Locked", false);
              Logger.recordOutput("Aim/XLocked", false);
              drive.setAimLocked(false);
            });
  }

  /**
   * For autos: stand still and turn the robot's back (the shooter) to our hub. Ends once it's
   * aimed (AIM LOCKED), or after {@code timeoutSeconds} so a bad aim can't freeze the auto.
   * Uses the drive, so put it as its own auto step, never as an event marker during a path.
   */
  public static Command aimAtHubInPlace(Drive drive, double timeoutSeconds) {
    Supplier<Rotation2d> hubHeading =
        () ->
            AimHelper.shootHeading(
                drive.getPose(), DriverStation.getAlliance().orElse(Alliance.Blue));
    return joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0, hubHeading, () -> false)
        .until(
            () ->
                Math.abs(hubHeading.get().minus(drive.getRotation()).getRadians())
                    < ALLOWABLE_ERROR_MARGIN_FOR_AIMING)
        .withTimeout(timeoutSeconds)
        // Stop the wheels when done. In an auto, the auto owns the drive, so the default
        // command can't stop them, and the robot would keep slowly turning while it shoots.
        .finallyDo(drive::stop);
  }

  /**
   * Hold-to-aim: the driver still drives with the left stick, and the robot turns its back
   * (the shooter) toward the target. In our alliance zone the target is our hub (SHOOT).
   * Anywhere else it's our wall (PASS). The math and the decision live in AimHelper.
   *
   * <p>While {@code shooting} is true and we're aimed at the hub, the wheels x-lock so we're hard
   * to push off target (like 2026). Passes don't x-lock. Pushing the left stick always breaks
   * the x-lock, so the driver can escape a defender.
   */
  public static Command autoAim(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier shooting) {
    return joystickDriveAtAngle(
            drive,
            xSupplier,
            ySupplier,
            () -> {
              Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
              Pose2d pose = drive.getPose();
              Logger.recordOutput("Aim/Mode", AimHelper.aimMode(pose, alliance).name());
              Logger.recordOutput("Aim/DistanceMeters", AimHelper.aimDistance(pose, alliance));
              return AimHelper.aimHeading(pose, alliance);
            },
            // X-lock only while shooting at the hub (not while passing).
            () ->
                shooting.getAsBoolean()
                    && AimHelper.aimMode(
                            drive.getPose(), DriverStation.getAlliance().orElse(Alliance.Blue))
                        == AimHelper.AimMode.SHOOT)
        .beforeStarting(() -> Logger.recordOutput("Aim/Active", true))
        .finallyDo(() -> Logger.recordOutput("Aim/Active", false));
  }

  /**
   * Test auto (best with the robot on blocks): drive all four wheels straight forward slowly
   * until the encoders say each wheel turned exactly ONE full rotation, then stop.
   *
   * <p>Put a tape mark on a wheel first. If the wheel stops right back where it started, the
   * gear ratio in DriveConstants is right. If it turns MORE than once, the real gearing is
   * faster than the code thinks (real turns = 5.5 / real gear ratio, with our 5.5:1 code).
   */
  public static Command wheelOneRotationTest(Drive drive) {
    double[] start = new double[4];
    DoubleSupplier averageWheelRadians =
        () -> {
          double[] now = drive.getWheelRadiusCharacterizationPositions();
          double total = 0.0;
          for (int i = 0; i < 4; i++) {
            total += Math.abs(now[i] - start[i]);
          }
          return total / 4.0;
        };
    // Slow, so it stops close to exactly one turn: 0.2 m/s at the wheel edge.
    double speedMetersPerSec = 0.05;
    return Commands.run(
            () -> {
              drive.runVelocity(new ChassisSpeeds(speedMetersPerSec, 0.0, 0.0));
              Logger.recordOutput(
                  "Test/WheelTurns", averageWheelRadians.getAsDouble() / (2.0 * Math.PI));
            },
            drive)
        .beforeStarting(
            () ->
                System.arraycopy(drive.getWheelRadiusCharacterizationPositions(), 0, start, 0, 4))
        .until(() -> averageWheelRadians.getAsDouble() >= 2.0 * Math.PI)
        .finallyDo(
            () -> {
              drive.stop();
              System.out.printf(
                  "Wheel turn test: encoders say %.3f turns. Check the tape mark on the wheel.%n",
                  averageWheelRadians.getAsDouble() / (2.0 * Math.PI));
            });
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>
   * This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
            () -> {
              drive.runCharacterization(0.0);
            },
            drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runCharacterization(voltage);
              velocitySamples.add(drive.getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                () -> {
                  var rotation = drive.getRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

}
