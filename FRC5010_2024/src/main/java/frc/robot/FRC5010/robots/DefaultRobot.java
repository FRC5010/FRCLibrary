// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.List;

import org.frc5010.common.arch.GenericMechanism;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.constants.DrivePorts;
import org.frc5010.common.constants.GenericDrivetrainConstants;
import org.frc5010.common.mechanisms.Drive;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.sensors.gyro.NavXGyro;
import org.frc5010.common.subsystems.AprilTagPoseSystem;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class DefaultRobot extends GenericRobot {
	private GenericDrivetrainConstants driveConstants;
	private GenericMechanism drive;
	private AprilTagPoseSystem vision;

	public DefaultRobot() {
		super();
		driveConstants = new GenericDrivetrainConstants();
		GenericGyro gyro = new NavXGyro(SPI.Port.kMXP);

		List<DrivePorts> motorPorts = new ArrayList<>();

		drive = new Drive(vision, gyro, Drive.Type.DIFF_DRIVE, motorPorts, driveConstants, "");
	}

	@Override
	public void configureButtonBindings(Controller driver, Controller operator) {
	}

	@Override
	public void setupDefaultCommands(Controller driver, Controller operator) {
		drive.setupDefaultCommands(driver, operator);
	}

	@Override
	protected void initRealOrSim() {
	}

	@Override
	public void initAutoCommands() {
		drive.initAutoCommands();
	}

	@Override
	public Command generateAutoCommand(Command autoCommand) {
		return drive.generateAutoCommand(autoCommand);
	}
}
