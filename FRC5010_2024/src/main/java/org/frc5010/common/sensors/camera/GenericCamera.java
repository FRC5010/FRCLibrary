// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.util.ArrayList;
import java.util.List;

import org.frc5010.common.vision.VisionConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Add your docs here. */
public abstract class GenericCamera {

	protected List<Runnable> updaters = new ArrayList<>();
	protected Transform3d cameraToRobot = new Transform3d();
	ShuffleboardTab visionTab;
	protected ShuffleboardLayout visionLayout;
	protected boolean updateValues = false;
	protected AprilTagFieldLayout fieldLayout;
	private int colIndex;
	private String name;

	public GenericCamera(String name, int colIndex, AprilTagFieldLayout fieldLayout, Transform3d cameraToRobot) {
		this.colIndex = colIndex;
		this.name = name;
		this.fieldLayout = fieldLayout;
		this.cameraToRobot = cameraToRobot;
		visionTab = Shuffleboard.getTab(VisionConstants.SBTabVisionDisplay);
		visionLayout = visionTab
				.getLayout("Camera " + this.name, BuiltInLayouts.kGrid)
				.withSize(2, 5)
				.withPosition(this.colIndex, 0);
		visionLayout.addBoolean("Has Target", this::hasValidTarget);
		visionLayout.addDouble("Target Yaw", this::getTargetYaw);
		visionLayout.addDouble("Target Pitch", this::getTargetPitch);
		visionLayout.addDouble("Target Area", this::getTargetArea);
		visionLayout.addDouble("Latency", this::getLatency);
	}

	/**
	 * Updates the state of the object by running all registered updaters.
	 *
	 * This method iterates over the list of updaters and calls the `run()` method
	 * on each one.
	 * This allows for the object to be updated based on the current state of the
	 * program.
	 *
	 * @throws NullPointerException if any of the updaters are null
	 */
	public void update() {
		for (Runnable updater : updaters) {
			updater.run();
		}
	}

	/**
	 * Registers a Runnable updater to the list of updaters.
	 *
	 * @param updater the Runnable updater to be added
	 */
	public void registerUpdater(Runnable updater) {
		updaters.add(updater);
	}

	/**
	 * Removes the specified updater from the list of updaters.
	 *
	 * @param updater the updater to be removed
	 */
	public void unregisterUpdater(Runnable updater) {
		updaters.remove(updater);
	}

	public abstract boolean hasValidTarget();

	public abstract double getTargetYaw();

	public abstract double getTargetPitch();

	public abstract double getTargetArea();

	public abstract double getLatency();

	public abstract Pose3d getRobotPose();

	public abstract Pose3d getRobotToTargetPose();
}
