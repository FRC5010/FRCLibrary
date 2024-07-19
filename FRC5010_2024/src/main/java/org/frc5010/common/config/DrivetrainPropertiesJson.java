// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;

/** Add your docs here. */
public class DrivetrainPropertiesJson {
	String directory;

	public void createDriveTrain(GenericRobot robot) {
		robot.addSubsystem("Drivetrain", new YAGSLSwerveDrivetrain(null, null, null, directory, null));
	}
}
