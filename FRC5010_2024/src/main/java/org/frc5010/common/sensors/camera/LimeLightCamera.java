// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.vision.LimelightHelpers;
import org.frc5010.common.vision.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class LimeLightCamera extends GenericCamera {
	Optional<PoseEstimate> poseEstimate = Optional.empty();
	Supplier<GenericGyro> gyroSupplier;
	BooleanSupplier megatagChooser;

	public LimeLightCamera(String name, int colIndex,
			BooleanSupplier megatagChooser) {
		super("limelight-" + name, colIndex, new Transform3d());
		this.megatagChooser = megatagChooser;
	}

		public LimeLightCamera(String name, int colIndex) {
		super("limelight-" + name, colIndex, new Transform3d());
	}

	protected Optional<PoseEstimate> getRobotPoseEstimateM1() {

		Optional<PoseEstimate> poseEstimate = processPoseEstimate(
			Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue(name)));
		if (poseEstimate.isPresent() && null != poseEstimate.get().pose && null != gyroSupplier) {
			SmartDashboard.putNumber("MT1 Angle", poseEstimate.get().pose.getRotation().getDegrees());
			gyroSupplier.get().setAngle(poseEstimate.get().pose.getRotation().getDegrees());
		}

		return poseEstimate;
	}

	protected Optional<PoseEstimate> getRobotPoseEstimateM2() {
		Optional<PoseEstimate> poseEstimate = processPoseEstimate(
				Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)));
		if (poseEstimate.isPresent() && poseEstimate.get().avgTagDist < 3) {
			getRobotPoseEstimateM1();
		}
		return poseEstimate;
	}

	private Optional<PoseEstimate> processPoseEstimate(Optional<PoseEstimate> poseEstimate) {
		if (poseEstimate.isPresent()) {
			Pose2d pose = poseEstimate.get().pose;

			if (poseEstimate.get().tagCount == 0) {
				pose = null;
			}

			if (null != gyroSupplier && Math.abs(gyroSupplier.get().getRate()) > 180) {
				pose = null;
			}

			poseEstimate.get().pose = pose;
		}
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
		return poseEstimate.map(it -> it.timestampSeconds).orElse(0.0);
	}

	@Override
	public Optional<Pose3d> getRobotPose() {
		return Optional.ofNullable(new Pose3d(poseEstimate.map(it -> it.pose).orElse(null)));
	}

	@Override
	public Optional<Pose3d> getRobotToTargetPose() {
		return Optional.empty();
	}

	public LimeLightCamera setGyroSupplier(Supplier<GenericGyro> gyroSupplier) {
		this.gyroSupplier = gyroSupplier;
		return this;
	}
}
