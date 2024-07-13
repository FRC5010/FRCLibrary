// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.frc5010.common.arch.GenericRobot;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

/** 
 * RobotParser is used to parse JSON configuration files to build a robot. 
*/
public class RobotParser {
	private static DriveteamControllersJson controllersJson;
	private static Map<String, DriveteamControllerConfiguration> controllersMap;
	private static VisionPropertiesJson visionJson;
	private static DrivetrainPropertiesJson driveTrainJson;

	public RobotParser(String robotDirectory, GenericRobot robot) throws IOException {
		File directory = new File(Filesystem.getDeployDirectory(), robotDirectory);
		checkDirectory(directory);
		controllersJson = new ObjectMapper().readValue(new File(directory, "controllers.json"), DriveteamControllersJson.class);
		controllersMap = new HashMap<>();
		for (int i = 0; i < controllersJson.controllers.length; i++) {
			File controllerFile = new File(directory, "controllers/" + controllersJson.controllers[i]);
			assert controllerFile.exists();
			DriveteamControllerJson controller = new ObjectMapper()
				.readValue(controllerFile, DriveteamControllerJson.class);	
			
			// Start a controller config	
			DriveteamControllerConfiguration controllerConfig = new DriveteamControllerConfiguration();	
			controllerConfig.setPort(controller.port);
			
			// Read in the axes
			for (int j = 0; j < controller.axis.length; j++) {
				File axisFile = new File(directory, "controllers/axis/" + controller.axis[j]);
				assert axisFile.exists();
				DriveteamControllerAxisJson axis = new ObjectMapper()
					.readValue(axisFile, DriveteamControllerAxisJson.class);
				controllerConfig.addAxis(axis);
			}
			controllersMap.put(controller.name, controllerConfig);
		}
		// visionJson = new ObjectMapper().readValue(new File(directory, "vision.json"), VisionPropertiesJson.class);
		// driveTrainJson = new ObjectMapper().readValue(new File(directory, "drivetrain.json"), DrivetrainPropertiesJson.class);	
	}

	/**
	 * Method to check the existence of specific JSON configuration files in the provided directory.
	 *
	 * @param  directory  the directory to check for JSON configuration files
	 */
	private void checkDirectory(File directory)
	{
	  assert new File(directory, "controller.json").exists();
	//   assert new File(directory, "vision.json").exists();
	//   assert new File(directory, "drivetrain.json").exists();
	}
  
	/**
	 * Method to create the robot.
	 *
	 * @param  robot	description of parameter
	 * @return         	description of return value
	 */
	public void createRobot(GenericRobot robot) {
		controllersJson.createControllers(robot, controllersMap);
		// driveTrainJson.createDriveTrain(robot);
		// visionJson.createCameraSystem(robot);
	}
}
