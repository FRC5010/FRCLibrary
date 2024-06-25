// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import org.frc5010.common.sensors.camera.GenericCamera;

/** Add your docs here. */
public class VisibleTargetSystem extends CameraSystem {
	boolean hasTargets = false;

	public VisibleTargetSystem(GenericCamera camera) {
		super(camera);
		camera.registerUpdater(() -> hasTargets = camera.hasValidTarget());
		camera.registerUpdater(() -> camera.getTargetYaw());
		camera.registerUpdater(() -> camera.getTargetPitch());
	}

	@Override
	public void updateCameraInfo() {
		camera.update();
	}

}
