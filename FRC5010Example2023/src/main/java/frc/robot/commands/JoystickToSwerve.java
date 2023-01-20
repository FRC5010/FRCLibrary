// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.Controller;
import frc.robot.FRC5010.drive.SwerveDrivetrain;
import frc.robot.mechanisms.SwerveDriveMech;

public class JoystickToSwerve extends CommandBase {
  /** Creates a new JoystickToSwerve. */
  private SwerveDrivetrain swerveDrive;
  private Joystick driver;
  private Supplier<Double> xSpdFunction, ySpdFunction, turnSpdFunction;
  private Supplier<Boolean> fieldOrientedDrive;
  private SlewRateLimiter xLimiter, yLimiter, turnLimiter; 

  public JoystickToSwerve(SwerveDrivetrain swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turnSpdFunction, Supplier<Boolean> fieldOrientedDrive) {
    this.swerveDrive = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turnSpdFunction = turnSpdFunction;
    this.fieldOrientedDrive = fieldOrientedDrive;

    this.xLimiter = new SlewRateLimiter(SwerveDrivetrain.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(SwerveDrivetrain.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turnLimiter = new SlewRateLimiter(SwerveDrivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    
    addRequirements(this.swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get values on sticks and deadzone them
    
    double xSpeed = (xSpdFunction.get());
    double ySpeed = (ySpdFunction.get());

    double turnSpeed = (turnSpdFunction.get());

    // limit power
    xSpeed = xLimiter.calculate(xSpeed) * SwerveDrivetrain.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveDrivetrain.kTeleDriveMaxSpeedMetersPerSecond;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveDrivetrain.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // convert to chassis speed class
    ChassisSpeeds chassisSpeeds;
    // 
    if(fieldOrientedDrive.get()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, 
        ySpeed, 
        turnSpeed, 
        swerveDrive.getHeading()
      );
    }else{
      chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turnSpeed);
    }

    // convert chassis speed into modules speeds
    SwerveModuleState[] moduleStates = SwerveDrivetrain.m_kinematics.toSwerveModuleStates(chassisSpeeds);

    // output each module speed into subsystem
    swerveDrive.setModuleStates(moduleStates);

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
