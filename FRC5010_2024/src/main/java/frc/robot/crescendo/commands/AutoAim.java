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
  Transform3d targetPose;

  /** Creates a new AutoAim. */
  public AutoAim(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, Drive drive) {

    targetPose = Constants.Field.SHOT_POSE;

    this.robotPose = drive.getDrivetrain().getPoseEstimator();
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drive = drive;

    addRequirements(pivotSubsystem, shooterSubsystem);
  }

  public AutoAim(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, Drive drive,
      Transform3d targetPose) {
    this.targetPose = targetPose;

    this.robotPose = drive.getDrivetrain().getPoseEstimator();
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drive = drive;

    addRequirements(pivotSubsystem, shooterSubsystem);
  }

  private double computeYawBetweenTransforms(Transform3d from, Transform3d to) {
    Translation3d difference = from.getTranslation().minus(to.getTranslation());
    double yawAngle = Math.atan2(difference.getY(), difference.getX());
    yawAngle += Math.PI / 2;
    yawAngle = Math.toDegrees(yawAngle);
    return yawAngle;
  }

  private double computePitchBetweenTransforms(Transform3d from, Transform3d to) {
    Translation3d difference = from.getTranslation().minus(to.getTranslation());
    double hypotenousA = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));
    double pivotAngle = Math.atan2(difference.getZ(), hypotenousA);
    return Math.toDegrees(pivotAngle);
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

    double yaw = computeYawBetweenTransforms(pivotOrigin, targetPose);
    double pitch = computePitchBetweenTransforms(pivotOrigin, targetPose);

    pivotSubsystem.setReference(pitch);

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
