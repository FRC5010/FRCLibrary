// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;

public class JoystickToSwerve extends Command {
  /** Creates a new JoystickToSwerve. */
  private SwerveDrivetrain swerveDrive;
  private Joystick driver;
  private DoubleSupplier xSpdFunction, ySpdFunction, turnSpdFunction;
  private BooleanSupplier fieldOrientedDrive;

  public JoystickToSwerve(SwerveDrivetrain swerveSubsystem, DoubleSupplier xSpdFunction,
      DoubleSupplier ySpdFunction, DoubleSupplier turnSpdFunction, BooleanSupplier fieldOrientedDrive) {
    this.swerveDrive = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turnSpdFunction = turnSpdFunction;
    this.fieldOrientedDrive = fieldOrientedDrive;

    addRequirements(this.swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public void setTurnSpeedFunction(DoubleSupplier turnSpeedFunction) {
    turnSpdFunction = turnSpeedFunction;
  }

  public DoubleSupplier getTurnSpeedFunction() {
    return turnSpdFunction;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get values on sticks and deadzone them

    double xSpeed = (xSpdFunction.getAsDouble());
    double ySpeed = (ySpdFunction.getAsDouble());

    double turnSpeed = (turnSpdFunction.getAsDouble());

    // limit power
    xSpeed = xSpeed * swerveDrive.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    ySpeed = ySpeed * swerveDrive.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    turnSpeed = turnSpeed * swerveDrive.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();

    // convert to chassis speed class
    ChassisSpeeds chassisSpeeds;
    //
    // System.out.println(swerveDrive.getGyroRate());
    double gyroRate = Units.degreesToRadians(swerveDrive.getGyroRate()) * 0.01;
    Rotation2d correctedRotation = swerveDrive.getHeading()
        .minus(new Rotation2d(gyroRate));

    if (fieldOrientedDrive.getAsBoolean()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed,
          ySpeed,
          turnSpeed,
          correctedRotation);
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
    }

    // convert chassis speed into modules speeds
    // SwerveModuleState[] moduleStates =
    // SwerveDrivetrain.m_kinematics.toSwerveModuleStates(chassisSpeeds);

    // output each module speed into subsystem
    swerveDrive.drive(chassisSpeeds);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
