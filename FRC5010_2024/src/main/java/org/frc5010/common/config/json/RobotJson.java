// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import java.io.File;
import java.io.IOException;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.constants.GenericDrivetrainConstants;

import com.fasterxml.jackson.databind.ObjectMapper;

/** Add your docs here. */
public class RobotJson {
	public String userConfig = "competitionMode.json";
	public String logLevel = "COMPETITION";
	public String driveType = "YAGSL_SWERVE_DRIVE";
	public double trackWidth = 0.0;
	public double wheelBase = 0.0;
	public double wheelDiameter = 0.0;
	public double physicalMaxSpeed = 0.0;
	public double driveMotorGearRatio = 1.0;
	public String[] mechanismDefinitions;

	public void configureRobot(GenericRobot robot, File directory) throws IOException {
		GenericRobot.setLoggingLevel(LogLevel.valueOf(logLevel));
		GenericDrivetrainConstants drivetrainConstants = robot.getDrivetrainConstants();
		drivetrainConstants.setTrackWidth(trackWidth);
		drivetrainConstants.setWheelBase(wheelBase);
		drivetrainConstants.setWheelDiameter(wheelDiameter);
		drivetrainConstants.setkPhysicalMaxSpeedMetersPerSecond(physicalMaxSpeed);

		UserModeJson userModeJson = new ObjectMapper().readValue(new File(directory, userConfig), UserModeJson.class);
		drivetrainConstants.setkTeleDriveMaxSpeedMetersPerSecond(userModeJson.maxSpeed);
		drivetrainConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(userModeJson.maxAngularSpeed);
		drivetrainConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(userModeJson.maxAcceleration);
		drivetrainConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(userModeJson.maxAngularAcceleration);
		drivetrainConstants.setkDriveMotorGearRatio(driveMotorGearRatio);
	}

	public void readMechanismDefinitionss(GenericRobot robot) {
	}
}
