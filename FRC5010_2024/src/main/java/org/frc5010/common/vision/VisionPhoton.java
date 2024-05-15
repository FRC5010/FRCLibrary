// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.frc5010.common.drive.pose.DrivetrainPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class VisionPhoton extends VisionSystem {
  protected PhotonPoseEstimator poseEstimator;
  protected PhotonCamera camera;
  protected PoseStrategy strategy;
  protected AprilTagFieldLayout fieldLayout;
  protected Pose2d referencePose = new Pose2d(0, 0, new Rotation2d(0));
  protected DrivetrainPoseEstimator drivetrainPoseEstimator = null;

  /** Creates a new LimeLightVision. */

  // makes a new limelight that is vertical, portrait
  public VisionPhoton(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      PoseStrategy strategy,
      DrivetrainPoseEstimator drivetrainPoseEstimator,
      Transform3d cameraToRobot) {
    super(name, colIndex, fieldLayout);
    this.cameraToRobot = cameraToRobot;
    this.fieldLayout = fieldLayout;
    this.strategy = strategy;
    camera = new PhotonCamera(name);
    poseEstimator = new PhotonPoseEstimator(fieldLayout, strategy, camera, cameraToRobot);
    setDrivetrainPoseEstimator(drivetrainPoseEstimator);
  }

  @Override
  public void update() {
    rawValues.clearValues();
    updateViaNetworkTable(camera, poseEstimator, name);
  }

  public void updateViaNetworkTable(
      PhotonCamera camera, PhotonPoseEstimator poseEstimator, String path) {
    if (null != drivetrainPoseEstimator) {
      if (strategy == PoseStrategy.CLOSEST_TO_REFERENCE_POSE) {
        poseEstimator.setReferencePose(drivetrainPoseEstimator.getCurrentPose());
      }
      if (strategy == PoseStrategy.CLOSEST_TO_LAST_POSE) {
        poseEstimator.setLastPose(drivetrainPoseEstimator.getCurrentPose());
      }
    }
    var camResult = camera.getLatestResult();
    Pose3d robotPoseEstInit = new Pose3d();
    PhotonTrackedTarget targetInit = null;
    double deltaTimeInit = Timer.getFPGATimestamp() - (camResult.getLatencyMillis() / 1000.0);
    if (camResult.hasTargets()) {
      targetInit = camResult.getBestTarget();
      Optional<EstimatedRobotPose> result = poseEstimator.update();

      if (result.isPresent()
          && result.get().estimatedPose != null
          && targetInit.getPoseAmbiguity() < 0.5) {
        robotPoseEstInit = result.get().estimatedPose;
        deltaTimeInit = result.get().timestampSeconds;
        referencePose = robotPoseEstInit.toPose2d();
      } else {
        robotPoseEstInit = null;
      }
      SmartDashboard.putNumber(camera.getName() + " ambiguity", targetInit.getPoseAmbiguity());
    }
    Pose2d robotPoseEst = null == robotPoseEstInit ? null : robotPoseEstInit.toPose2d();
    double deltaTime = deltaTimeInit;
    var target = targetInit;
    updateValues(
        rawValues,
        () -> target.getYaw(),
        () -> target.getPitch(),
        () -> target.getArea(),
        () -> camResult.hasTargets(),
        () -> deltaTime,
        () -> target.getFiducialId(),
        () -> fieldLayout.getTagPose(target.getFiducialId()).orElse(null),
        () -> robotPoseEst);
  }

  public DrivetrainPoseEstimator getDrivetrainPoseEstimator() {
    return drivetrainPoseEstimator;
  }

  public void setDrivetrainPoseEstimator(DrivetrainPoseEstimator drivetrainPoseEstimator) {
    this.drivetrainPoseEstimator = drivetrainPoseEstimator;
  }

  // name is assigned in the constructor, and will give you the correct limelight
  // table
  // aka use name whenever you use getTable()

  public int getTargetFiducial() {
    return rawValues.getFiducialId();
  }

  public void setLight(boolean on) {
    camera.setLED(on ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void flashLight() {
    camera.setLED(VisionLEDMode.kBlink);
  }

  public boolean isLightOn() {
    return camera.getLEDMode().compareTo(VisionLEDMode.kOn) == 0;
  }

  public void toggleLight() {
    camera.setLED(
        camera.getLEDMode() == VisionLEDMode.kOff ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  public void setPipeline(int pipeline) {
    camera.setPipelineIndex(pipeline);
  }

  public void setSnapshotMode(int snapVal) {
    camera.takeInputSnapshot();
  }
}
