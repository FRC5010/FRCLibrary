// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import java.util.ArrayList;
import java.util.List;

import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.Controller.Axis;

/**
 * A class for storing configuration data for a single controller.
 * 
 * @apiNote port: The port number of the controller.
 * @apiNote axes: The axes of the controller.
 */
public class DriveteamControllerConfiguration {
	private int port;
	private List<DriveteamControllerAxisJson> axes = new ArrayList<>();

	/**
	 * Configures a new Controller object with the specified port and axes.
	 *
	 * @return a new Controller object configured with the specified port and axes
	 */
	public Controller configureController() {
		Controller controller = new Controller(port);
		axes.forEach(it -> {
			Axis axis = controller.createAxis(it.channel)
					.limit(it.limit)
					.scale(it.scale)
					.curvePower(it.curvePower);
			if (it.rate != 0) {
				axis = axis.rate(it.rate);
			}
			axis = axis.negate(it.invert)
					.deadzone(it.deadband);
			controller.setAxis(it.channel, axis);
		});

		return controller;
	}

	/**
	 * A method to set the port number of the controller.
	 *
	 * @param  port  the port number to be set
	 */
	public void setPort(int port) {
		this.port = port;
	}

	/**
	 * Adds a DriveteamControllerAxisJson object to the list of axes.
	 *
	 * @param  axis  the DriveteamControllerAxisJson object to be added
	 */
	public void addAxis(DriveteamControllerAxisJson axis) {
		axes.add(axis);
	}
}
