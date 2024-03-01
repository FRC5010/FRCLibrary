// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.commands.JoystickToSwerve;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.crescendo.Constants;
import frc.robot.crescendo.PivotSubsystem;
import frc.robot.crescendo.ShooterSubsystem;
import frc.robot.crescendo.Targeting2024;

public class AutoAim extends Command {

  DrivetrainPoseEstimator robotPose;
  ShooterSubsystem shooterSubsystem;
  PivotSubsystem pivotSubsystem;
  Drive drive;
  SwerveDrivetrain drivetrain;
  Transform3d targetPose;
  Targeting2024 targetingSystem;
  Controller driverXboxController;
  Supplier<JoystickToSwerve> driveCommand;
  DoubleSupplier originalTurnSpeed;

  /** Creates a new AutoAim. */

  public AutoAim(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, Drive drive,
      Targeting2024 targetSystem, Supplier<JoystickToSwerve> driveCommand) {

    drivetrain = (SwerveDrivetrain) drive.getDrivetrain();
    this.robotPose = drivetrain.getPoseEstimator();
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drive = drive;
    this.targetingSystem = targetSystem;
    this.driveCommand = driveCommand;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose.setDisableVisionUpdate(true);
    originalTurnSpeed = driveCommand.get().getTurnSpeedFunction();
    driveCommand.get().setTurnSpeedFunction(() -> targetingSystem.getTargetingRotation(() -> robotPose.getCurrentPose3d().getTranslation(), drivetrain) );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform3d pivotOrigin = new Transform3d(
                robotPose.getCurrentPose3d().getTranslation().plus(Constants.Physical.PIVOT_ORIGIN_OFFSET.getTranslation()), new Rotation3d());
    double pivotAngle = Targeting2024.getPivotAngleToTarget(new Pose3d(pivotOrigin.getTranslation(), pivotOrigin.getRotation()), Constants.Field.BLUE_SHOT_POSE);
    SmartDashboard.putNumber("Shooting Pivot Angle", pivotAngle);
    pivotSubsystem.setReference(pivotAngle + 44.1);
      // shooterSubsystem.setReference(4000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveCommand.get().setTurnSpeedFunction(originalTurnSpeed);
    robotPose.setDisableVisionUpdate(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
