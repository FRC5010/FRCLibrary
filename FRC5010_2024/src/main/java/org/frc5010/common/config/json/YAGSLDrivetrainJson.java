// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import java.io.File;
import java.io.IOException;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.DrivetrainConfiguration;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.subsystems.AprilTagPoseSystem;

import com.fasterxml.jackson.databind.ObjectMapper;

/** Add your docs here. */
public class YAGSLDrivetrainJson extends DrivetrainPropertiesJson {
	public String directory = "";
	public double turningMotorGearRatio = 1.0;
	public String[] driveModules;

	public void readDrivetrainConfiguration(GenericRobot robot, File baseDirectory) throws IOException {
		SwerveConstants swerveConstants = new SwerveConstants(robot.getDrivetrainConstants());
		for (int i = 0; i < driveModules.length; i++) {
			File moduleFile = new File(baseDirectory, "drive_modules/" + driveModules[i]);
			String moduleName = driveModules[i].substring(0, driveModules[i].indexOf(".json"));
			assert moduleFile.exists();
			YAGSLDriveModuleJson module = new ObjectMapper().readValue(moduleFile, YAGSLDriveModuleJson.class);
			MotorFeedFwdConstants feedFwdConstants = new MotorFeedFwdConstants(module.s, module.v, module.a);
			swerveConstants.getSwerveModuleConstants().addDriveMotorFF(moduleName, feedFwdConstants);
		}
		robot.setDrivetrainConstants(swerveConstants);
		return;
	};

	@Override
	public void createDriveTrain(GenericRobot robot) {
		AprilTagPoseSystem atSystem = (AprilTagPoseSystem) robot.getSubsystem(CameraConfigurationJson.APRIL_TAG);
		robot.addSubsystem(DrivetrainConfiguration.DRIVE_TRAIN,
				new YAGSLSwerveDrivetrain(robot.getMechVisual(), robot.getDrivetrainConstants(), turningMotorGearRatio, directory, atSystem));
	}
}
