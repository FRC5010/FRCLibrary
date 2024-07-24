// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import java.util.ArrayList;
import java.util.List;

import org.frc5010.common.subsystems.AprilTagPoseSystem;
import org.frc5010.common.vision.AprilTags;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DrivePoseEstimator {
	protected GenericPose poseTracker;
	protected AprilTagPoseSystem vision;
	private final Field2d field2d;
	private List<Pose2d> tagPoses = new ArrayList<>();
	private boolean disableVisionUpdate = false;

	public DrivePoseEstimator(GenericPose poseTracker, AprilTagPoseSystem vision) {
		this.poseTracker = poseTracker;
		this.vision = vision;
		field2d = poseTracker.getField();

		ShuffleboardTab tab = Shuffleboard.getTab("Pose");
		tab.addString("Pose (X,Y)", this::getFormattedPose).withPosition(11, 0);
		tab.addDoubleArray("Robot Pose3d", () -> getCurrentPose3dArray()).withPosition(11, 1);

		tab.addNumber("Pose Degrees", () -> (getCurrentPose().getRotation().getDegrees()))
				.withPosition(11, 2);
		tab.add("Pose Field", field2d).withPosition(0, 0).withSize(11, 5);

		for (AprilTag at : vision.getFieldLayout().getTags()) {
			if (at.pose.getX() != 0 && at.pose.getY() != 0 && at.pose.getZ() != 0) {
				field2d.getObject("Field Tag " + at.ID).setPose(at.pose.toPose2d());
				AprilTags.poseToID.put(at.pose.toPose2d(), at.ID);
				tagPoses.add(at.pose.toPose2d());
			}
		}

	}

	public void setDisableVisionUpdate(boolean disable) {
		disableVisionUpdate = disable;
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

	public void update() {
		poseTracker.updateLocalMeasurements();
		if (!disableVisionUpdate) {
			List<String> cameras = vision.getCameraNames();
			boolean visionUpdated = false;
			for (String camera : cameras) {
				SmartDashboard.putBoolean(camera, false);
				Pose3d robotPose = vision.getRobotPose3d(camera);
				double poseDistance = vision.getDistanceToTarget(camera);
				if (null != robotPose) {
					double imageCaptureTime = vision.getLatency(camera);
					visionUpdated = true;
					SmartDashboard.putBoolean(camera, true);
					poseTracker.updateVisionMeasurements(
							robotPose.toPose2d(), imageCaptureTime, vision.getStdVector(poseDistance));
				}
			}
			SmartDashboard.putBoolean("April Tag Pose Updating", visionUpdated);
		}
		field2d.setRobotPose(getCurrentPose());
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

	public Rotation2d getGyroRotation2d() {
		// System.out.println(poseTracker.getGyroRotation2d());
		return poseTracker.getGyroRotation2d();
	}

	public int getClosestTagToRobot() {
		return AprilTags.poseToID.get(getCurrentPose().nearest(tagPoses));
	}

	public void setTargetPoseOnField(Pose2d targetPose, String targetName) {
		field2d.getObject(targetName).setPose(targetPose);
	}

	public Pose3d getPoseFromClosestTag() {
		Pose3d targetPose =
			vision.getFieldLayout().getTagPose(getClosestTagToRobot()).orElse(getCurrentPose3d());
		field2d.getObject("Closest Tag").setPose(targetPose.toPose2d());
		return targetPose;
	  }
	
}
