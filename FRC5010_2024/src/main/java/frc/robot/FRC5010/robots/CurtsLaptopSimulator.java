// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.DrivetrainConfiguration;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * To setup a copy of this class Copy to a new class and rename the constructor
 * In RobotFactory add
 * constants and a switch-case for your class Start PhotonVision and get the
 * camera name (java -jar
 * photonvision.jar) Start the robot simulation In Glass - find theO
 * NetworkTables->Preferences->WhoAmI and set it to your robot name Also change
 * the LaptopCamera
 * name from photon vision Stop the simulator, restart and it should start your
 * robot code
 */
public class CurtsLaptopSimulator extends GenericRobot {
	SwerveConstants swerveConstants;
	GenericDrivetrain drivetrain;

	public CurtsLaptopSimulator(String directory) {
		super(directory);
		drivetrain = (GenericDrivetrain) getSubsystem(DrivetrainConfiguration.DRIVE_TRAIN);
	}

	@Override
	public void configureButtonBindings(Controller driver, Controller operator) {
	}

	@Override
	public void setupDefaultCommands(Controller driver, Controller operator) {
		drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
	}

	@Override
	protected void initRealOrSim() {
	}

	@Override
	public void initAutoCommands() {
		drivetrain.setAutoBuilder();
	}

	@Override
	public Command generateAutoCommand(Command autoCommand) {
		return drivetrain.generateAutoCommand(autoCommand);
	}
}
