// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.Constants;
import frc.robot.crescendo.PivotSubsystem;
import frc.robot.crescendo.ShooterSubsystem;
import frc.robot.crescendo.TargetingSystem;

public class PredefinedAutoShot extends Command {

  Rotation2d targetYaw;
  double targetPivot;
  double targetShooterSpeedTop;
  double targetShooterSpeedBottom;

  double velocity = 0;
  double yawOffset = 0;
  double pivotOffset = 0;

  PivotSubsystem pivotSubsystem;
  ShooterSubsystem shooterSubsystem;

  // Targeting
  TargetingSystem targetingSystem;
  Pose2d targetingPose;

  /** Creates a new PredefinedAutoShot. */
  public PredefinedAutoShot(double yawAngle, double pivotAngle, double shooterSpeed, PivotSubsystem pivotSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.targetYaw = Rotation2d.fromDegrees(yawAngle);
    this.targetPivot = pivotAngle;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.targetShooterSpeedTop = shooterSpeed;
    this.targetShooterSpeedBottom = shooterSpeed;
  }

  public PredefinedAutoShot(Pose2d expectedPose, TargetingSystem targetingSystem, PivotSubsystem pivotSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.targetShooterSpeedTop = Constants.Physical.TOP_SHOOTING_SPEED;
    this.targetShooterSpeedBottom = Constants.Physical.BOTTOM_SHOOTING_SPEED;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.targetingSystem = targetingSystem;
    this.targetingPose = expectedPose;

  }

  public PredefinedAutoShot withVelocity(double velocity) {
    this.velocity = velocity;
    return this;
  }

  public PredefinedAutoShot withYawOffset(double offset) {
    this.yawOffset = offset;
    return this;
  }

  public PredefinedAutoShot withPivotOffset(double offset) {
    this.pivotOffset = offset;
    return this;
  }

  public PredefinedAutoShot withShooterVelocity(double top, double bottom) {
    this.targetShooterSpeedTop = top;
    this.targetShooterSpeedBottom = bottom;
    return this;
  }

  private void calculateTargetingValues() {
    Pose3d robotPose = new Pose3d(
        new Translation3d(targetingPose.getTranslation().getX(), targetingPose.getTranslation().getY(), 0.0),
        new Rotation3d());
    this.targetPivot = TargetingSystem.interpolatePivotAngle(
        robotPose,
        targetingSystem.getTarget());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (targetingSystem != null && targetingPose != null) {
      calculateTargetingValues();
    }

    //pivotSubsystem.setReference(targetPivot + pivotOffset);
    shooterSubsystem.setShooterReference(targetShooterSpeedTop, targetShooterSpeedBottom);
    PPHolonomicDriveController
        .setRotationTargetOverride(() -> Optional.of(targetYaw.plus(Rotation2d.fromDegrees(yawOffset))));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PPHolonomicDriveController.setRotationTargetOverride(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
