// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.chargedup.PivotSubsystem;
import frc.robot.crescendo.Constants;
import frc.robot.crescendo.ShooterSubsystem;

public class AutoAim extends Command {
 
  DrivetrainPoseEstimator robotPose;
  ShooterSubsystem shooterSubsystem;
  PivotSubsystem pivotSubsystem;
  Drive drive;
  Pose3d targetPose = new Pose3d(new Translation3d(Constants.Field.SHOT_X, Constants.Field.SHOT_Y, Constants.Field.SHOT_Z), new Rotation3d());
  /** Creates a new AutoAim. */
  public AutoAim(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, Drive drive) {
    
    this.robotPose = drive.getDrivetrain().getPoseEstimator();
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drive = drive;

    double shooterSpeed = 6000;
    
    addRequirements(pivotSubsystem, shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform3d difference = robotPose.getCurrentPose3d().minus(targetPose);
    double hypotenousA = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));
    double pivotAngle = Math.atan2(difference.getZ(), hypotenousA);
    double robotRawAngle = Math.atan2(difference.getY(), difference.getX());
    double robotAngle;
    if (robotRawAngle < 0) {
      robotAngle = -90 - robotRawAngle;
    } else {
      robotAngle = 90 - robotRawAngle;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
