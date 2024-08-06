// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.frc5010.common.sensors.camera.GenericCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class AprilTagPoseSystem extends CameraSystem {
	protected Map<String, Optional<Pose3d>> robotPose3ds = new HashMap<>();
	protected Map<String, Optional<Pose3d>> targetPose3ds = new HashMap<>();
	protected Map<String, Double> latencies = new HashMap<>();
	protected List<GenericCamera> cameras = new ArrayList<>();
	protected AprilTagFieldLayout fieldLayout;

	public AprilTagPoseSystem(AprilTagFieldLayout fieldLayout) {
		super(null);
		this.fieldLayout = fieldLayout;
	}

	public AprilTagPoseSystem(GenericCamera camera, AprilTagFieldLayout fieldLayout) {
		super(camera);
		this.fieldLayout = fieldLayout;
		addCamera(camera);
	}

	public void addCamera(GenericCamera camera) {
		if (null == this.camera) {
			this.camera = camera;
		}
		cameras.add(camera);
		camera.registerUpdater(() -> robotPose3ds.put(camera.name(), camera.getRobotPose()));
		camera.registerUpdater(() -> targetPose3ds.put(camera.name(), camera.getRobotToTargetPose()));
		camera.registerUpdater(() -> latencies.put(camera.name(), camera.getLatency()));
	}

	public Optional<Pose3d> getRobotPose3d(String name) {
		return Optional.ofNullable(robotPose3ds.get(name)).orElse(Optional.empty());
	}

	public Double getLatency(String name) {
		return latencies.get(name);
	}

	public List<String> getCameraNames() {
		return cameras.stream().map(c -> c.name()).toList();
	}

	@Override
	public void updateCameraInfo() {
		for (GenericCamera camera : cameras) {
			camera.update();
		}
	}

	public AprilTagFieldLayout getFieldLayout() {
		return fieldLayout;
	}

	public double getDistanceToTarget(String camera) {
		return Optional.ofNullable(targetPose3ds.get(camera))
				.map(it -> it.get().getTranslation().getNorm())
				.orElse(Double.MAX_VALUE);
	}

	@Override
	public double getDistanceToTarget() {
		double shortestDistance = Double.MAX_VALUE;
		for (GenericCamera camera : cameras) {
			double distance = getDistanceToTarget(camera.name());
			if (distance < shortestDistance || shortestDistance == 0.0) {
				shortestDistance = distance;
			}
		}
		return shortestDistance;
	}

	public Vector<N3> getStdVector(double distance) {
		double calib = distance * 0.15;
		return VecBuilder.fill(calib, calib, Units.degreesToRadians(5 * distance));
	}
}
