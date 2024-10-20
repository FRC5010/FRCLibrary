// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.common.arch.GenericCommand;
import frc.robot.crescendo.Constants;
import frc.robot.crescendo.FeederSubsystem.NoteState;
import frc.robot.crescendo.PivotSubsystem;
import frc.robot.crescendo.ShooterSubsystem;
import frc.robot.crescendo.TargetingSystem;

public class PredefinedAutoShot extends GenericCommand {

  Rotation2d targetYaw;
  double targetPivot;
  Optional<Double> targetShooterSpeed = Optional.empty();

  boolean pivotEnabled = false;
  boolean yawEnabled = false;
  boolean spinupEnabled = false;

  double velocity = 0;
  double yawOffset = 0;
  double pivotOffset = 0;

  

  PivotSubsystem pivotSubsystem;
  ShooterSubsystem shooterSubsystem;

  // Targeting
  TargetingSystem targetingSystem;
  Pose2d targetingPose;

  Supplier<NoteState> noteStateSupplier = null;

  /** Creates a new PredefinedAutoShot. */
  public PredefinedAutoShot(double yawAngle, double pivotAngle, double shooterSpeed, PivotSubsystem pivotSubsystem,
      ShooterSubsystem shooterSubsystem, Supplier<NoteState> noteState) {
    this.targetYaw = Rotation2d.fromDegrees(yawAngle);
    this.targetPivot = pivotAngle;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.targetShooterSpeed = Optional.empty();
    this.noteStateSupplier = noteState;

  }

  public PredefinedAutoShot(Pose2d expectedPose, TargetingSystem targetingSystem, PivotSubsystem pivotSubsystem,
      ShooterSubsystem shooterSubsystem, Supplier<NoteState> noteState) {
    this.targetShooterSpeed = Optional.empty();
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.targetingSystem = targetingSystem;
    this.targetingPose = expectedPose;
    this.noteStateSupplier = noteState;

  }

  public PredefinedAutoShot withVelocity(double velocity) {
    this.velocity = velocity;
    return this;
  }

  public PredefinedAutoShot withYawOffset(double offset) {
    this.yawOffset = offset;
    return enableYaw();
  }

  public PredefinedAutoShot withPivotOffset(double offset) {
    this.pivotOffset = offset;
    this.pivotEnabled = true;
    return enablePivot();
  }

  public PredefinedAutoShot enableSpinup() {
    this.spinupEnabled = true;
    return this;
  }

  public PredefinedAutoShot enableYaw() {
    this.yawEnabled = true;
    return this;
  }

  public PredefinedAutoShot enablePivot() {
    this.pivotEnabled = true;
    return this;
  }

  public PredefinedAutoShot withShooterVelocity(double speed) {
    this.targetShooterSpeed = Optional.of(speed);
    return enableSpinup();
  }

  private void calculateTargetingValues() {
    Pose3d robotPose = new Pose3d(
        new Translation3d(targetingPose.getTranslation().getX(), targetingPose.getTranslation().getY(), 0.0),
        new Rotation3d());
    this.targetPivot = TargetingSystem.interpolatePivotAngle(
        robotPose,
        targetingSystem.getTarget());

    if (spinupEnabled) {
      this.targetShooterSpeed = Optional.of(TargetingSystem.interpolateShooterSpeed( robotPose.getTranslation().toTranslation2d().getDistance(targetingSystem.getTarget().getTranslation().toTranslation2d()) ));
    }

    this.targetYaw = Rotation2d.fromDegrees(targetingSystem.getYawToTarget(robotPose));

  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    targetYaw = new Rotation2d();
    targetPivot = 0;

    if (targetingSystem != null && targetingPose != null) {
      calculateTargetingValues();
    }

    if (pivotEnabled) {
      pivotSubsystem.setReference(targetPivot + pivotOffset);
    }

    if (yawEnabled) {
      PPHolonomicDriveController
          .setRotationTargetOverride(() -> Optional.of(targetYaw.plus(Rotation2d.fromDegrees(yawOffset))));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetShooterSpeed.isPresent() && (noteStateSupplier == null || noteStateSupplier.get() == NoteState.Loaded
        || noteStateSupplier.get() == NoteState.Shooting)) {
      shooterSubsystem.setShooterReference(targetShooterSpeed.get(), targetShooterSpeed.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    PPHolonomicDriveController.setRotationTargetOverride(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (null != noteStateSupplier) {
      return noteStateSupplier.get() == NoteState.Empty;
    }
    return false;
  }
}
