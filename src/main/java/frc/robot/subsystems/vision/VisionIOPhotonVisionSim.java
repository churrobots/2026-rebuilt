// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.simCameraStreams;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.sim.FieldBoundaries;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  // Each camera gets its own sim world, so it can be given only the tags it has a clear
  // line of sight to (hubs block the view).
  private final VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name         The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    visionSim = new VisionSystemSim(name);
    visionSim.addAprilTags(aprilTagLayout);

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    // TODO: make this configurable, maybe even import config.json after calibration
    cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(80));
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    // Video for the dashboard: the processed view, with the field drawn as lines
    cameraSim.enableRawStream(false);
    cameraSim.enableProcessedStream(simCameraStreams);
    cameraSim.enableDrawWireframe(simCameraStreams);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Pose2d robotPose = poseSupplier.get();

    // Only let this camera "see" tags that no hub is hiding.
    Translation2d cameraSpot =
        new Pose3d(robotPose).transformBy(robotToCamera).toPose2d().getTranslation();
    List<AprilTag> visibleTags = new ArrayList<>();
    for (AprilTag tag : aprilTagLayout.getTags()) {
      Translation2d tagSpot = tag.pose.toPose2d().getTranslation();
      if (!FieldBoundaries.isBlocked(cameraSpot, tagSpot)) {
        visibleTags.add(tag);
      }
    }
    visionSim.clearAprilTags();
    visionSim.addAprilTags(
        new AprilTagFieldLayout(
            visibleTags, aprilTagLayout.getFieldLength(), aprilTagLayout.getFieldWidth()));

    visionSim.update(robotPose);
    super.updateInputs(inputs);
  }
}
