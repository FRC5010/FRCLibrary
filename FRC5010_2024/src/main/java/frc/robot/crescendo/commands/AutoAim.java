// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.crescendo.PivotSubsystem;
import frc.robot.crescendo.Constants;
import frc.robot.crescendo.ShooterSubsystem;
import frc.robot.crescendo.Targeting2024;

public class AutoAim extends Command {

  DrivetrainPoseEstimator robotPose;
  ShooterSubsystem shooterSubsystem;
  PivotSubsystem pivotSubsystem;
  Drive drive;
  Transform3d targetPose;
  Targeting2024 targetingSystem;



  /** Creates a new AutoAim. */


  public AutoAim(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, Drive drive, Targeting2024 targetSystem) {

    this.robotPose = drive.getDrivetrain().getPoseEstimator();
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drive = drive;
    this.targetingSystem = targetSystem;

    addRequirements(pivotSubsystem, shooterSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation3d robotTranslation = robotPose.getCurrentPose3d().getTranslation();
    // TODO: Have to account for robot rotation just like with camera
    Transform3d pivotOrigin = new Transform3d(
        robotTranslation.plus(Constants.Physical.PIVOT_ORIGIN_OFFSET.getTranslation()), new Rotation3d());
    ChassisSpeeds robotVelocity = ((SwerveDrivetrain)drive.getDrivetrain()).getChassisSpeeds();
    robotVelocity = robotVelocity != null ? robotVelocity : new ChassisSpeeds();
    double angleSpeed = targetingSystem.getAnglePowerToTarget(robotVelocity);
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, angleSpeed, drive.getDrivetrain().getHeading());
    drive.getDrivetrain().drive(chassisSpeeds);
    SmartDashboard.putNumber("Angle Speed", angleSpeed);
    // TODO: Rotate Drive to Target and Allow for Joystick Translation

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
