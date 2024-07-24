// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.frc5010.common.vision.AprilTags;
import org.frc5010.common.vision.VisionSystem;

/** Add your docs here. */
public class DrivetrainPoseEstimator {
  private VisionSystem vision;
  private final Field2d field2d;
  private final GenericPose poseTracker;
  private boolean disableVisionUpdate = false;

  private int closestTagToRobot;
  private Pose3d closestTargetToRobot;
  private List<Pose2d> tagPoses = new ArrayList<>();

  public DrivetrainPoseEstimator(GenericPose poseTracker, VisionSystem vision) {
    this.poseTracker = poseTracker;
    this.vision = vision;
    field2d = poseTracker.getField();

    ShuffleboardTab tab = Shuffleboard.getTab("Pose");
    tab.addString("Pose (X,Y)", this::getFormattedPose).withPosition(11, 0);
    tab.addDoubleArray("Robot Pose3d", () -> getCurrentPose3dArray()).withPosition(11, 1);

    tab.addNumber("Pose Degrees", () -> (getCurrentPose().getRotation().getDegrees()))
        .withPosition(11, 2);
    tab.add(field2d).withPosition(0, 0).withSize(11, 5);

    for (AprilTag at : vision.getFieldLayout().getTags()) {
      if (at.pose.getX() != 0 && at.pose.getY() != 0 && at.pose.getZ() != 0) {
        field2d.getObject("Field Tag " + at.ID).setPose(at.pose.toPose2d());
        AprilTags.poseToID.put(at.pose.toPose2d(), at.ID);
        tagPoses.add(at.pose.toPose2d());
      }
    }
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f)", pose.getX(), pose.getY());
  }

  public Pose2d getCurrentPose() {
    return poseTracker.getCurrentPose();
  }

  public Pose3d getCurrentPose3d() {
    return new Pose3d(poseTracker.getCurrentPose());
  }

  public void setDisableVisionUpdate(boolean disable) {
    disableVisionUpdate = disable;
  }

  public Pose3d getProjectedPose3d(double time, ChassisSpeeds robotFieldSpeed) {
    Transform3d movement =
        new Transform3d(
            robotFieldSpeed.vxMetersPerSecond * time,
            robotFieldSpeed.vyMetersPerSecond * time,
            0,
            new Rotation3d());
    return getCurrentPose3d().plus(movement);
  }

  public double[] getCurrentPose3dArray() {
    Pose3d pose = getCurrentPose3d();
    Quaternion rotation = pose.getRotation().getQuaternion();
    return new double[] {
      pose.getX(),
      pose.getY(),
      pose.getZ(),
      rotation.getW(),
      rotation.getX(),
      rotation.getY(),
      rotation.getZ()
    };
  }

  public Rotation2d getGyroRotation2d() {
    // System.out.println(poseTracker.getGyroRotation2d());
    return poseTracker.getGyroRotation2d();
  }

  public int getClosestTagToRobot() {
    closestTagToRobot = AprilTags.poseToID.get(getCurrentPose().nearest(tagPoses));
    return closestTagToRobot;
  }

  public Pose3d getPoseFromClosestTag() {
    Pose3d targetPose =
        vision.getFieldLayout().getTagPose(getClosestTagToRobot()).orElse(getCurrentPose3d());
    field2d.getObject("Closest Tag").setPose(targetPose.toPose2d());
    return targetPose;
  }

  public Pose3d getPoseFromClosestVisionTarget() {
    closestTargetToRobot = updatePoseFromClosestVisionTarget();
    return closestTargetToRobot;
  }

  public Pose3d updatePoseFromClosestVisionTarget() {
    double angleYaw = vision.getAngleX();
    double anglePitch = vision.getAngleY();
    double distance = -vision.getDistance();
    Transform3d cameraTransform = vision.getCameraToRobot();

    Pose3d targetPose = null;
    if (null != cameraTransform) {
      Pose2d currentPose2d = poseTracker.getCurrentPose();
      Pose3d currentPose3d =
          new Pose3d(
              currentPose2d.getX(),
              currentPose2d.getY(),
              0.0,
              new Rotation3d(0.0, 0.0, currentPose2d.getRotation().getRadians()));

      Pose3d cameraPose = currentPose3d.transformBy(cameraTransform);
      if (vision.isValidTarget()) {
        targetPose =
            cameraPose.transformBy(
                new Transform3d(
                    Math.cos(Units.degreesToRadians(-angleYaw)) * distance,
                    Math.sin(Units.degreesToRadians(-angleYaw)) * distance,
                    Math.sin(Units.degreesToRadians(-anglePitch)) * distance,
                    new Rotation3d(0, 0, 0)));
        field2d.getObject("Closest Target").setPose(targetPose.toPose2d());
      }

      field2d.getObject("Target Camera").setPose(cameraPose.toPose2d());
    }
    return targetPose;
  }

  public void setTargetPoseOnField(Pose2d targetPose, String targetName) {
    field2d.getObject(targetName).setPose(targetPose);
  }

  /*
   * Perform all periodic pose estimation tasks.
   *
   */

  public void update() {

    poseTracker.updateLocalMeasurements();
    if (!disableVisionUpdate) {
      Map<String, Pose2d> poses = vision.getRawValues().getRobotPoses();
      Map<String, Double> poseDistances = vision.getRawValues().getPoseDistances();
      boolean visionUpdated = false;
      for (String camera : poses.keySet()) {
        SmartDashboard.putBoolean(camera, false);
        Pose2d robotPose = poses.get(camera);
        Double poseDistance = poseDistances.get(camera);
        if (null != robotPose) {
          double imageCaptureTime = vision.getRawValues().getLatency(camera);

          if (vision.getRawValues().getFiducialIds().get(camera) > 0) {
            visionUpdated = true;
            SmartDashboard.putBoolean(camera, true);
            poseTracker.updateVisionMeasurements(
                robotPose, imageCaptureTime, vision.getStdVector(poseDistance));
          }
        }
      }
      SmartDashboard.putBoolean("April Tag Pose Updating", visionUpdated);
    }
    field2d.setRobotPose(getCurrentPose());
  }

  /**
   * Force the pose estimator to a particular pose. This is useful for indicating to the software
   * when you have manually moved your robot in a particular position on the field (EX: when you
   * place it on the field at the start of the match).
   *
   * @param pose
   */
  public void resetToPose(Pose2d pose) {
    poseTracker.resetToPose(pose);
  }
}
