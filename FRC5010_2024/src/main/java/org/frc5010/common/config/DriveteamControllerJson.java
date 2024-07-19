// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

/** 
 * A simple class to store configuration data for a single driveteam controller.
 * @apiNote name: A public field to store the name of a driveteam controller.
 * @apiNote port: A public field to store the port number of a driveteam controller.
 * @apiNote axis: A public field to store an array of axis file names for a driveteam controller.
 */
public class DriveteamControllerJson {
	public String name;
	public int port;
	public String[] axis;
}
