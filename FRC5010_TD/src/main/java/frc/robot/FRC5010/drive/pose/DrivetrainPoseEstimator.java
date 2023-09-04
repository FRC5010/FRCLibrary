// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.pose;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionSystem;

/** Add your docs here. */
public class DrivetrainPoseEstimator {
  private VisionSystem vision;
  private final Field2d field2d = new Field2d();
  private final GenericPose poseTracker;
  private int closestTagToRobot;
  private List<Pose2d> tagPoses = new ArrayList<>();

  public DrivetrainPoseEstimator(GenericPose poseTracker, VisionSystem vision) {
    this.poseTracker = poseTracker;
    this.vision = vision;

    ShuffleboardTab tab = Shuffleboard.getTab("Pose");
    tab.addString("Pose (X,Y)", this::getFormattedPose).withPosition(0, 4);
    tab.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 4);
    tab.add(field2d);

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

  public Rotation2d getGyroRotation2d() {
    // System.out.println(poseTracker.getGyroRotation2d());
    return poseTracker.getGyroRotation2d();
  }

  public int getClosestTagToRobot() {
    return closestTagToRobot;
  }

  public Pose2d getPoseFromClosestTag() {
    Pose2d targetPose = vision.getFieldLayout().getTagPose(closestTagToRobot).map(it -> it.toPose2d())
        .orElse(getCurrentPose());
    field2d.getObject("Closest Tag").setPose(targetPose);
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

    for (Pose2d robotPose : vision.getRawValues().getRobotPoses()) {
      if (null != robotPose) {
        double imageCaptureTime = vision.getRawValues().getLatency();
        // field2d.getObject("MyRobot" +
        // ((VisionValuesPhotonCamera)vision.getRawValues()).getFiducialId()).setPose(robotPose);
        if (RobotState.isDisabled()
            || 0.5 > robotPose.getTranslation().getDistance(poseTracker.getCurrentPose().getTranslation())) {
          poseTracker.updateVisionMeasurements(robotPose, imageCaptureTime);
        }
      }
    }
    field2d.setRobotPose(getCurrentPose());

    closestTagToRobot = AprilTags.poseToID.get(getCurrentPose().nearest(tagPoses));
  }

  /**
   * Force the pose estimator to a particular pose. This is useful for indicating
   * to the software
   * when you have manually moved your robot in a particular position on the field
   * (EX: when you
   * place it on the field at the start of the match).
   *
   * @param pose
   */
  public void resetToPose(Pose2d pose) {
    poseTracker.resetToPose(pose);
  }
}
