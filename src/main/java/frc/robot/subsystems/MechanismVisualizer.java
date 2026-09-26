package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

/** Publishes a simple mechanism schematic and ordered 3D component poses. */
public final class MechanismVisualizer extends SubsystemBase {
  private final IntakeArm intakeArm;
  private final IntakeRoller intakeRoller;
  private final Spindexer spindexer;
  private final Feeder feeder;
  private final Shooter shooter;

  private final LoggedMechanism2d schematic = new LoggedMechanism2d(1.0, 1.0);
  private final LoggedMechanismLigament2d intakeArmLigament;
  private final LoggedMechanismLigament2d intakeRollerLigament;
  private final LoggedMechanismLigament2d spindexerLigament;
  private final LoggedMechanismLigament2d feederLigament;
  private final LoggedMechanismLigament2d shooterLigament;

  public MechanismVisualizer(
      IntakeArm intakeArm,
      IntakeRoller intakeRoller,
      Spindexer spindexer,
      Feeder feeder,
      Shooter shooter) {
    this.intakeArm = intakeArm;
    this.intakeRoller = intakeRoller;
    this.spindexer = spindexer;
    this.feeder = feeder;
    this.shooter = shooter;

    intakeArmLigament =
        schematic
            .getRoot("IntakePivot", 0.72, 0.20)
            .append(
                new LoggedMechanismLigament2d(
                    "IntakeArm",
                    MechanismVisualizationConstants.INTAKE_ARM_LENGTH_METERS,
                    0.0,
                    8.0,
                    new Color8Bit(Color.kWhite)));
    intakeRollerLigament =
        intakeArmLigament
            .append(
                new LoggedMechanismLigament2d(
                    "IntakeRoller", 0.08, 0.0, 8.0, new Color8Bit(Color.kGreen)));
    spindexerLigament =
        schematic
            .getRoot("SpindexerPivot", 0.50, 0.40)
            .append(
                new LoggedMechanismLigament2d(
                    "Spindexer", 0.14, 0.0, 8.0, new Color8Bit(Color.kWhite)));
    feederLigament =
        schematic
            .getRoot("FeederPivot", 0.32, 0.58)
            .append(
                new LoggedMechanismLigament2d(
                    "Feeder", 0.10, 0.0, 8.0, new Color8Bit(Color.kGreen)));
    shooterLigament =
        schematic
            .getRoot("ShooterPivot", 0.22, 0.78)
            .append(
                new LoggedMechanismLigament2d(
                    "Shooter", 0.12, 0.0, 8.0, new Color8Bit(Color.kRed)));
  }

  @Override
  public void periodic() {
    intakeArmLigament.setAngle(intakeArm.getVisualizationAngle());
    intakeRollerLigament.setAngle(
        Math.toDegrees(intakeRoller.getVisualizationSpinRadians()));
    spindexerLigament.setAngle(
        Math.toDegrees(spindexer.getComponentPose().getRotation().getZ()));
    feederLigament.setAngle(Math.toDegrees(feeder.getComponentPose().getRotation().getY()));
    shooterLigament.setAngle(Math.toDegrees(shooter.getComponentPose().getRotation().getX()));

    Logger.recordOutput("Mechanisms/Schematic", schematic);
    // Component order must match model_0.glb through model_4.glb in the robot asset.
    Logger.recordOutput(
        "Mechanisms/ComponentPoses",
        new Pose3d[] {
          intakeArm.getComponentPose(),
          intakeRoller.getComponentPose(),
          spindexer.getComponentPose(),
          feeder.getComponentPose(),
          shooter.getComponentPose()
        });
  }
}
