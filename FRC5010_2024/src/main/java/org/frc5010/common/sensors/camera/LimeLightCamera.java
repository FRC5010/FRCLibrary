// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.vision.LimelightHelpers;
import org.frc5010.common.vision.LimelightHelpers.PoseEstimate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class LimeLightCamera extends GenericCamera {
	PoseEstimate poseEstimate = null;
	Supplier<GenericGyro> gyroSupplier;
	BooleanSupplier megatagChooser;

	public LimeLightCamera(String name, int colIndex, AprilTagFieldLayout fieldLayout, Transform3d cameraToRobot,
			BooleanSupplier megatagChooser) {
		super("limelight-" + name, colIndex, fieldLayout, cameraToRobot);
		this.megatagChooser = megatagChooser;
	}

	protected PoseEstimate getRobotPoseEstimateM1() {

		PoseEstimate poseEstimate = processPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(name));
		if (null != poseEstimate.pose) {
			SmartDashboard.putNumber("MT1 Angle", poseEstimate.pose.getRotation().getDegrees());
			gyroSupplier.get().setAngle(poseEstimate.pose.getRotation().getDegrees());
		}

		return poseEstimate;
	}

	protected PoseEstimate getRobotPoseEstimateM2() {
		PoseEstimate poseEstimate = processPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name));
		if (poseEstimate.avgTagDist < 3) {
			getRobotPoseEstimateM1();
		}
		return poseEstimate;
	}

	private PoseEstimate processPoseEstimate(PoseEstimate poseEstimate) {
		Pose2d pose = poseEstimate.pose;

		if (poseEstimate.tagCount == 0) {
			pose = null;
		}

		if (null != gyroSupplier && Math.abs(gyroSupplier.get().getRate()) > 180) {
			pose = null;
		}

		poseEstimate.pose = pose;
		return poseEstimate;
	}

	public void setPoseEstimationChooser(BooleanSupplier chooser) {
		megatagChooser = chooser;
	}

	@Override
	public void update() {
		if (hasValidTarget()) {
			poseEstimate = megatagChooser.getAsBoolean() ? getRobotPoseEstimateM1() : getRobotPoseEstimateM2();
		}
	}

	@Override
	public boolean hasValidTarget() {
		return LimelightHelpers.getLimelightNTTableEntry(name, "tv").getDouble(0) == 1;
	}

	@Override
	public double getTargetYaw() {
		return LimelightHelpers.getLimelightNTDouble(name, "tx");
	}

	@Override
	public double getTargetPitch() {
		return LimelightHelpers.getLimelightNTDouble(name, "ty");
	}

	@Override
	public double getTargetArea() {
		return LimelightHelpers.getLimelightNTDouble(name, "ta");
	}

	@Override
	public double getLatency() {
		return poseEstimate.timestampSeconds;
	}

	@Override
	public Pose3d getRobotPose() {
		return new Pose3d(poseEstimate.pose);
	}

	@Override
	public Pose3d getRobotToTargetPose() {
		return null;
	}
}
