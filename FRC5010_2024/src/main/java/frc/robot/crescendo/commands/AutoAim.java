// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.commands.JoystickToSwerve;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.drive.swerve.YAGSLSwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.crescendo.Constants;
import frc.robot.crescendo.FeederSubsystem;
import frc.robot.crescendo.PivotSubsystem;
import frc.robot.crescendo.ShooterSubsystem;
import frc.robot.crescendo.TargetingSystem;

public class AutoAim extends Command {

  DrivetrainPoseEstimator robotPose;
  ShooterSubsystem shooterSubsystem;
  PivotSubsystem pivotSubsystem;
  Drive drive;
  SwerveDrivetrain drivetrain;
  Transform3d targetPose;
  TargetingSystem targetingSystem;
  Controller driverXboxController;
  Supplier<JoystickToSwerve> driveCommand;
  DoubleSupplier originalTurnSpeed;
  FeederSubsystem feederSubsystem;
  double cycleCounter = 0;
  boolean useAutoDrive = false;

  /** Creates a new AutoAim. */

  public AutoAim(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem,
      Drive drive,
      TargetingSystem targetSystem, Supplier<JoystickToSwerve> driveCommand) {

    drivetrain = (SwerveDrivetrain) drive.getDrivetrain();
    this.robotPose = drivetrain.getPoseEstimator();
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drive = drive;
    this.targetingSystem = targetSystem;
    this.driveCommand = driveCommand;
    this.feederSubsystem = feederSubsystem;
  }

  public AutoAim(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem,
      Drive drive,
      TargetingSystem targetSystem, boolean useAutoDrive) {
    this.useAutoDrive = useAutoDrive;
    drivetrain = (SwerveDrivetrain) drive.getDrivetrain();
    this.robotPose = drivetrain.getPoseEstimator();
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drive = drive;
    this.targetingSystem = targetSystem;
    this.feederSubsystem = feederSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose.setDisableVisionUpdate(true);
    if (driveCommand != null) {
      originalTurnSpeed = driveCommand.get().getTurnSpeedFunction();
      driveCommand.get().setTurnSpeedFunction(() -> targetingSystem.getTurnPower());
    } else if (useAutoDrive) {
      ((YAGSLSwerveDrivetrain) drive.getDrivetrain()).setAngleSupplier(() -> targetingSystem.getTurnPower());
    } 
    cycleCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform3d pivotOrigin = new Transform3d(
        robotPose.getCurrentPose3d().getTranslation().plus(Constants.Physical.PIVOT_ORIGIN_OFFSET.getTranslation()),
        new Rotation3d());
    double pivotAngle = targetingSystem.getPivotAngle();
    SmartDashboard.putNumber("Shooting Pivot Angle", pivotAngle);
    pivotSubsystem.setReference(pivotAngle);
    shooterSubsystem.setShooterReference(6000, 6000);
    double turnSpeed = targetingSystem.getTurnPower();
    if (!useAutoDrive && driveCommand == null) {

      drive.getDrivetrain().drive(new ChassisSpeeds(0, 0, turnSpeed * ((SwerveDrivetrain)drive.getDrivetrain()).getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond()));
    }
    SmartDashboard.putBoolean("Pivot Target", pivotSubsystem.isAtTarget());
    SmartDashboard.putBoolean("Shooter Target", shooterSubsystem.isAtTarget());
    SmartDashboard.putBoolean("Rotation Target", targetingSystem.isAtTargetYaw());
    if (pivotSubsystem.isAtTarget() && shooterSubsystem.isAtTarget() && targetingSystem.isAtTargetYaw()) {
      if (cycleCounter > 2 && Math.abs(turnSpeed) < 0.05) {
        feederSubsystem.feederStateMachine(-1.0);

      }
      cycleCounter++;

    } else {
      cycleCounter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (driveCommand != null) {
      driveCommand.get().setTurnSpeedFunction(originalTurnSpeed);
    } else {
      ((YAGSLSwerveDrivetrain) drive.getDrivetrain()).setAngleSupplier(null);
    }
    robotPose.setDisableVisionUpdate(false);
    feederSubsystem.setFeederSpeed(0);
    shooterSubsystem.setShooterReference(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
