// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.DriveteamControllerConfiguration;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class DriveteamControllersJson {
	public String[] controllers;

	/**
	 * Creates controllers for a given robot using the provided map of controller
	 * configurations.
	 *
	 * @param robot the robot to add the controllers to
	 * @param map   a map of controller configurations, where the key is the
	 *              controller name and the value is the configuration object
	 */
	public void createControllers(GenericRobot robot, Map<String, DriveteamControllerConfiguration> map) {
		map.keySet().forEach(it -> {
			robot.addController(it, map.get(it).configureController());
		});
	}

	public Map<String, DriveteamControllerConfiguration> readControllers(File directory)
			throws IOException, StreamReadException, DatabindException {
		Map<String, DriveteamControllerConfiguration> controllersMap = new HashMap<>();
		for (int i = 0; i < controllers.length; i++) {
			File controllerFile = new File(directory, "controllers/" + controllers[i]);
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
		return controllersMap;
	}

}
