// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.arch.GenericCommand;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.chargedup.IntakeSubsystem;

public class AutoIntake extends GenericCommand {
  SwerveDrivetrain drive;
  VisionSystem visionSystem;
  double lastAngleY;

  private final String X_SPEED = "X Speed";
  private final String Y_SPEED = "Y Speed";
  private final String ANGLE_SPEED = "Angle Speed";

  /** Creates a new AutoIntake. */
  public AutoIntake(SwerveDrivetrain drive, VisionSystem vision) {
    this.drive = drive;
    this.visionSystem = vision;

    values.declare(X_SPEED, -0.015);
    values.declare(Y_SPEED, 0.005);
    values.declare(ANGLE_SPEED, -0.005);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    lastAngleY = visionSystem.getAngleY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngleY = visionSystem.getAngleY();
    double currentAngleX = visionSystem.getAngleX();

    if (currentAngleY > lastAngleY + 5 || !visionSystem.isValidTarget()) {
      currentAngleX = 0;
      currentAngleY = 0;
    } else {
      lastAngleY = currentAngleY;
    }

    double xSpeed = currentAngleY * values.getDouble(X_SPEED);
    double ySpeed = currentAngleX * values.getDouble(Y_SPEED);
    double turnSpeed = currentAngleX * values.getDouble(ANGLE_SPEED);

    // limit power
    xSpeed = xSpeed * drive.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    ySpeed = ySpeed * drive.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    turnSpeed = turnSpeed * drive.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();

    //
    // System.out.println(swerveDrive.getGyroRate());
    double gyroRate = Units.degreesToRadians(drive.getGyroRate()) * 0.01;

   
    drive.drive(new ChassisSpeeds(xSpeed, ySpeed, turnSpeed));

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
