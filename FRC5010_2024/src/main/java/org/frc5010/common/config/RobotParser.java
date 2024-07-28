// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import java.io.File;
import java.io.IOException;
import java.util.Map;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.json.CameraConfigurationJson;
import org.frc5010.common.config.json.DriveteamControllersJson;
import org.frc5010.common.config.json.DrivetrainPropertiesJson;
import org.frc5010.common.config.json.RobotJson;
import org.frc5010.common.config.json.VisionPropertiesJson;
import org.frc5010.common.config.json.YAGSLDrivetrainJson;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * RobotParser is used to parse JSON configuration files to build a robot.
 */
public class RobotParser {
	private static DriveteamControllersJson controllersJson;
	private static Map<String, DriveteamControllerConfiguration> controllersMap;
	private static VisionPropertiesJson visionJson;
	private static Map<String, CameraConfigurationJson> camerasMap;
	private static DrivetrainPropertiesJson driveTrainJson;

	public RobotParser(String robotDirectory, GenericRobot robot) throws IOException {
		File directory = new File(Filesystem.getDeployDirectory(), robotDirectory);
		checkDirectory(directory);

		// Read in the robot configuration
		RobotJson robotJson = new ObjectMapper().readValue(new File(directory, "robot.json"), RobotJson.class);
		robotJson.configureRobot(robot, directory);

		// Read in the controllers
		controllersJson = new ObjectMapper().readValue(new File(directory, "controllers.json"),
				DriveteamControllersJson.class);
		controllersMap = controllersJson.readControllers(directory);

		// Read in the cameras
		visionJson = new ObjectMapper().readValue(new File(directory, "cameras.json"), VisionPropertiesJson.class);
		camerasMap = visionJson.readCameraSystem(directory);

		switch (robotJson.driveType) {
			case "YAGSL_SWERVE_DRIVE": {
				YAGSLDrivetrainJson yagslDriveTrainJson = new ObjectMapper()
						.readValue(new File(directory, "yagsl_drivetrain.json"), YAGSLDrivetrainJson.class);
				yagslDriveTrainJson.readDrivetrainConfiguration(robot, directory);
				driveTrainJson = yagslDriveTrainJson;
				break;
			}
			default: {
				driveTrainJson = new ObjectMapper()
						.readValue(new File(directory, "drivetrain.json"), DrivetrainPropertiesJson.class);
				driveTrainJson.readDrivetrainConfiguration(robot, directory);
				break;
			}
		}
		robotJson.readMechanismDefinitionss(robot);
	}

	/**
	 * Method to check the existence of specific JSON configuration files in the
	 * provided directory.
	 *
	 * @param directory the directory to check for JSON configuration files
	 */
	private void checkDirectory(File directory) {
		assert new File(directory, "controller.json").exists();
		assert new File(directory, "vision.json").exists();
		assert new File(directory, "drivetrain.json").exists();
	}

	/**
	 * Method to create the robot.
	 *
	 * @param robot description of parameter
	 * @return description of return value
	 */
	public void createRobot(GenericRobot robot) {
		controllersJson.createControllers(robot, controllersMap);
		visionJson.createCameraSystem(robot, camerasMap);
		driveTrainJson.createDriveTrain(robot);
	}
}
