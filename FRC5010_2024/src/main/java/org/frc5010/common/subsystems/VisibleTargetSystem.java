// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import org.frc5010.common.sensors.camera.GenericCamera;

import edu.wpi.first.math.geometry.Transform3d;

public class VisibleTargetSystem extends CameraSystem {
	boolean hasTargets = false;
	double targetHeight = 0;
	double targetPitch = 0;
	double targetYaw = 0;

	public VisibleTargetSystem(GenericCamera camera, double targetHeight) {
		super(camera);
		this.targetHeight = targetHeight;
		camera.registerUpdater(() -> hasTargets = camera.hasValidTarget());
		camera.registerUpdater(() -> targetYaw = camera.getTargetYaw());
		camera.registerUpdater(() -> targetPitch = camera.getTargetPitch());
	}

	/**
	 * A method to get the distance to the target.
	 *
	 * @return the distance to the target, or Double.MAX_VALUE if no valid target
	 */
	@Override
	public double getDistanceToTarget() {
		Transform3d camera2Robot = camera.getCameraToRobot();
		return hasTargets ? (targetHeight - camera2Robot.getTranslation().getZ())
				/ (Math.tan(Math.toRadians(targetPitch) + camera2Robot.getRotation().getY())
						* Math.cos(Math.toRadians(targetYaw)))
				+ camera2Robot.getTranslation().getNorm()
				: Double.MAX_VALUE;
	}
}
