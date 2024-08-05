// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.robot;

import org.frc5010.common.sensors.Controller;

import edu.wpi.first.wpilibj2.command.Command;

import org.frc5010.common.arch.GenericRobot;

/** Add your docs here. */
public class TestRobot extends GenericRobot {
	public TestRobot() {
		super();
	}

	@Override
	public Command generateAutoCommand(Command autoCommand) {
		return autoCommand;
	}

	@Override
	public void initAutoCommands() {
	}

	@Override
	public void setupDefaultCommands(Controller one, Controller two) {
	}

	@Override
	public void configureButtonBindings(Controller one, Controller two) {
	}
	
}
