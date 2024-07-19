// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import java.util.Map;

import org.frc5010.common.arch.GenericRobot;

public class DriveteamControllersJson {
	public String[] controllers;

	/**
	 * Creates controllers for a given robot using the provided map of controller configurations.
	 *
	 * @param  robot   the robot to add the controllers to
	 * @param  map     a map of controller configurations, where the key is the controller name and the value is the configuration object
	 */
	public void createControllers(GenericRobot robot,  Map<String, DriveteamControllerConfiguration> map) {
		map.keySet().forEach(it -> {
			robot.addController(it, map.get(it).configureController());
		});
	}
}
