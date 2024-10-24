// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frc5010.common.arch.GenericCommand;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.commands.JoystickToSwerve;
import org.frc5010.common.drive.pose.DrivePoseEstimator;
import org.frc5010.common.drive.swerve.SwerveDrivetrain;
import org.frc5010.common.mechanisms.Drive;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.sensors.gyro.GenericGyro;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.LogLevel;
import frc.robot.crescendo.Constants;
import frc.robot.crescendo.FeederSubsystem;
import frc.robot.crescendo.PivotSubsystem;
import frc.robot.crescendo.ShooterSubsystem;
import frc.robot.crescendo.TargetingSystem;

public class AutoAim extends GenericCommand {

  // Test Mode
  private String PIVOT_ANGLE_OVERRIDE = "Pivot Angle Override";
  private String SHOOTER_SPEED_OVERRIDE = "Shooter Speed Override";

  DrivePoseEstimator robotPose;
  ShooterSubsystem shooterSubsystem;
  PivotSubsystem pivotSubsystem;
  Drive drive;
  SwerveDrivetrain drivetrain;
  Transform3d targetPose;
  TargetingSystem targetingSystem;
  Controller driverXboxController;
  GenericGyro gyro;
  Supplier<JoystickToSwerve> driveCommand;
  DoubleSupplier originalTurnSpeed;
  FeederSubsystem feederSubsystem;
  double cycleCounter = 0;
  double timeoutCounter = 0;
  boolean useAutoDrive = false;
  boolean useShooterCamera = false;
  private double startTime, endTime, startAngle;
  private boolean accountForMovement = false;
  private boolean interpolatingYaw = false;
  private double turnSpeed;

  /** Creates a new AutoAim. */

  public AutoAim(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem,
      Drive drive,
      TargetingSystem targetSystem, GenericGyro gyro, Supplier<JoystickToSwerve> driveCommand,
      boolean accountForMovement, boolean interpolatingYaw) {

    drivetrain = (SwerveDrivetrain) drive.getDrivetrain();
    this.robotPose = drivetrain.getPoseEstimator();
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drive = drive;
    this.targetingSystem = targetSystem;
    this.driveCommand = driveCommand;
    this.feederSubsystem = feederSubsystem;
    this.gyro = gyro;
    this.accountForMovement = accountForMovement;
    this.interpolatingYaw = interpolatingYaw;
  }

  public AutoAim(PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem,
      Drive drive,
      TargetingSystem targetSystem, GenericGyro gyro, boolean useAutoDrive, boolean accountForMovement,
      boolean interpolatingYaw) {
    this.useAutoDrive = useAutoDrive;
    drivetrain = (SwerveDrivetrain) drive.getDrivetrain();
    this.robotPose = drivetrain.getPoseEstimator();
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.drive = drive;
    this.targetingSystem = targetSystem;
    this.feederSubsystem = feederSubsystem;
    this.gyro = gyro;
    this.accountForMovement = accountForMovement;
    this.interpolatingYaw = interpolatingYaw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    SmartDashboard.putNumber(PIVOT_ANGLE_OVERRIDE, 0.0);
    SmartDashboard.putNumber(SHOOTER_SPEED_OVERRIDE, 0.0);
    targetingSystem.setAccountForMovement(accountForMovement);
    robotPose.setDisableVisionUpdate(true);
    if (driveCommand != null) {
      originalTurnSpeed = driveCommand.get().getTurnSpeedFunction();
      turnSpeed = targetingSystem.getInitialStationaryTurnPower();
      driveCommand.get().setTurnSpeedFunction(() -> targetingSystem.getTurnPower());
      startTime = System.nanoTime();
      startAngle = gyro.getAngleZ();
    } else if (useAutoDrive) {
      PPHolonomicDriveController.setRotationTargetOverride(() -> targetingSystem.getRotationTarget(1.0));
    }
    cycleCounter = 0;
    timeoutCounter = 0;
    useShooterCamera = true;
    targetingSystem.setInterpolatingYaw(interpolatingYaw);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(robotPose.getCurrentPose().getX() - targetingSystem.getCurrentTarget().getX()) > 5.0) {
      targetingSystem.setTarget(TargetingSystem.getShuttleTarget(GenericRobot.getAlliance()));
    }

    if (targetingSystem.isAtTargetYaw() && endTime == 0.0) {
      endTime = System.nanoTime();
      SmartDashboard.putNumber("Yaw Interpolation Time", (System.nanoTime() - startTime) / 1_000_000_000.0);
      SmartDashboard.putNumber("Yaw Interpolation Angle Rotated", Math.abs(gyro.getAngleZ() - startAngle));
    }

    Transform3d pivotOrigin = new Transform3d(
        robotPose.getCurrentPose3d().getTranslation().plus(Constants.Physical.PIVOT_ORIGIN_OFFSET.getTranslation()),
        new Rotation3d());
    double pivotAngle = pivotSubsystem.HOME_LEVEL;

    Optional<Double> shootCamAngle = targetingSystem.getShooterCamAngle();

    // if (shootCamAngle.isPresent()) {
    //   pivotAngle = shootCamAngle.get();
    // } else {
    //   pivotAngle = targetingSystem.getPivotAngle();
    // }
      pivotAngle = targetingSystem.getPivotAngle();


    Optional<Double> yawCamAngle = targetingSystem.getShooterYawPower();
    

    boolean atTargetYaw = false;
    if (accountForMovement || interpolatingYaw) {
      turnSpeed = targetingSystem.getTurnPower();
      atTargetYaw = targetingSystem.isAtTargetYaw();
    } else if (yawCamAngle.isPresent()) {
      turnSpeed = yawCamAngle.get();
      atTargetYaw = targetingSystem.isAtTargetCameraYaw();
    } else {
      turnSpeed = targetingSystem.getTurnPower();
      atTargetYaw = targetingSystem.isAtTargetYaw();
    }

    SmartDashboard.putNumber("Turn Speed", turnSpeed);

    pivotSubsystem.setReference(pivotAngle);
    SmartDashboard.putNumber("Shooting Pivot Angle", pivotAngle);

    double shootingSpeed = targetingSystem.getShooterSpeed();
    shooterSubsystem.setShooterReference(shootingSpeed, shootingSpeed);

    if (!useAutoDrive && driveCommand == null) {
      drive.getDrivetrain().drive(new ChassisSpeeds(0, 0, turnSpeed * ((SwerveDrivetrain) drive.getDrivetrain())
          .getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond()));
    }

    SmartDashboard.putBoolean("Pivot Target", pivotSubsystem.isAtTarget());
    SmartDashboard.putBoolean("Shooter Target", shooterSubsystem.isAtTarget());
    SmartDashboard.putBoolean("Rotation Target", atTargetYaw);
    boolean ready = (pivotSubsystem.isAtTarget() && shooterSubsystem.isAtTarget() && atTargetYaw);
    SmartDashboard.putBoolean("Ready To Shoot", ready);
    feederSubsystem.getNoteState();
    feederSubsystem.setShotReadyness(ready);
    if (useAutoDrive || driveCommand == null) {
      if (ready || 100 < timeoutCounter) {
        if ((cycleCounter > 2 && Math.abs(turnSpeed) < 0.2) || 60 < timeoutCounter) {
          feederSubsystem.feederStateMachine(-1.0);
          if (LogLevel.DEBUG == RobotContainer.getLoggingLevel()) {
            SmartDashboard.putNumber("Shot Angle", pivotSubsystem.getPivotPosition());
            SmartDashboard.putNumber("Shot Reference", pivotSubsystem.getReference());
            SmartDashboard.putNumber("Shot Speed", shooterSubsystem.getTopVelocity());
            SmartDashboard.putNumber("Shot Distance to Speaker", targetingSystem.getFlatDistanceToTarget());
          }

        }
        cycleCounter++;

      } else {
        cycleCounter = 0;
      }
      timeoutCounter++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    if (driveCommand != null) {
      driveCommand.get().setTurnSpeedFunction(originalTurnSpeed);
    } else {
      PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty());
    }
    robotPose.setDisableVisionUpdate(false);
    feederSubsystem.feederStateMachine(0);
    shooterSubsystem.setShooterReference(0, 0);
    targetingSystem.setAccountForMovement(false);
    targetingSystem.setInterpolatingYaw(false);
    targetingSystem.setTarget(TargetingSystem.getSpeakerTarget(GenericRobot.getAlliance()));
    endTime = 0.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
