// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/** Add your docs here. */
public class PhotonVisionCamera extends GenericCamera {
	static VisionSystemSim visionSim = new VisionSystemSim("main");
	protected static boolean fieldRegisted = false;

	protected PhotonPoseEstimator poseEstimator;
	protected PhotonCamera camera;
	protected PoseStrategy strategy;
	protected AprilTagFieldLayout fieldLayout;
	protected Optional<PhotonTrackedTarget> target = Optional.empty();
	protected PhotonPipelineResult camResult;
	protected SimCameraProperties cameraProp = new SimCameraProperties();
	protected PhotonCameraSim cameraSim;
	protected Supplier<Pose2d> poseSupplier;

	public PhotonVisionCamera(String name,
			int colIndex,
			AprilTagFieldLayout fieldLayout,
			PoseStrategy strategy,
			Transform3d cameraToRobot,
			Supplier<Pose2d> poseSupplier) {
		super(name, colIndex, cameraToRobot);
		this.cameraToRobot = cameraToRobot;
		this.fieldLayout = fieldLayout;
		this.strategy = strategy;
		this.poseSupplier = poseSupplier;
		camera = new PhotonCamera(name);
		poseEstimator = new PhotonPoseEstimator(fieldLayout, strategy, camera, cameraToRobot);

		if (Robot.isSimulation()) {
			visionSim.addAprilTags(fieldLayout);

			// A 640 x 480 camera with a 100 degree diagonal FOV.
			cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
			// Approximate detection noise with average and standard deviation error in
			// pixels.
			cameraProp.setCalibError(0.25, 0.08);
			// Set the camera image capture framerate (Note: this is limited by robot loop
			// rate).
			cameraProp.setFPS(40);
			// The average and standard deviation in milliseconds of image data latency.
			cameraProp.setAvgLatencyMs(35);
			cameraProp.setLatencyStdDevMs(5);

			cameraSim = new PhotonCameraSim(camera, cameraProp);
			visionSim.addCamera(cameraSim, cameraToRobot);

			if (!fieldRegisted) {
				fieldRegisted = true;
				visionTab.add("Vision Field", visionSim.getDebugField());
			}
		}
	}

	@Override
	public void update() {
		camResult = camera.getLatestResult();
		if (camResult.hasTargets()) {
			target = Optional.ofNullable(camResult.getBestTarget());
		}
		if (Robot.isSimulation()) {
			visionSim.update(poseSupplier.get());
			visionSim.resetRobotPose(poseSupplier.get());
		}
	}

	@Override
	public boolean hasValidTarget() {
		return camResult.hasTargets();
	}

	@Override
	public double getTargetYaw() {
		return target.map(t -> t.getYaw()).orElse(Double.MAX_VALUE);
	}

	@Override
	public double getTargetPitch() {
		return target.map(t -> t.getPitch()).orElse(Double.MAX_VALUE);
	}

	@Override
	public double getTargetArea() {
		return target.map(t -> t.getArea()).orElse(Double.MAX_VALUE);
	}

	@Override
	public double getLatency() {
		return Timer.getFPGATimestamp() - (camResult.getLatencyMillis() / 1000.0);
	}

	@Override
	public Optional<Pose3d> getRobotPose() {
		Pose3d robotPoseEst = null;
		if (null != poseSupplier) {
			if (strategy == PoseStrategy.CLOSEST_TO_REFERENCE_POSE) {
				poseEstimator.setReferencePose(poseSupplier.get());
			}
			if (strategy == PoseStrategy.CLOSEST_TO_LAST_POSE) {
				poseEstimator.setLastPose(poseSupplier.get());
			}
		}
		if (target.isPresent()) {
			Optional<EstimatedRobotPose> result = poseEstimator.update();

			if (result.isPresent()
					&& result.get().estimatedPose != null
					&& target.get().getPoseAmbiguity() < 0.5) {
				robotPoseEst = result.get().estimatedPose;
			}
		}
		return Optional.ofNullable(robotPoseEst);
	}

	@Override
	public Optional<Pose3d> getRobotToTargetPose() {
		Pose3d targetPoseEst = null;
		if (target.isPresent()) {
			if (target.get().getFiducialId() != 0) {
				targetPoseEst = fieldLayout.getTagPose(target.get().getFiducialId()).orElse(null);
			}
		}
		return Optional.ofNullable(targetPoseEst);
	}
}
