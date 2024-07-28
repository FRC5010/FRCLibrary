// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.commands.DefaultDriveCommand;
import org.frc5010.common.drive.pose.DrivePoseEstimator;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public abstract class GenericDrivetrain extends GenericSubsystem {
  protected Mechanism2d mechVisual;
  protected DrivePoseEstimator poseEstimator;
  protected boolean isFieldOrientedDrive = true;

  public GenericDrivetrain(Mechanism2d mechVisual) {
    this.mechVisual = mechVisual;
    Shuffleboard.getTab("Drive")
        .addBoolean("Field Oriented", () -> isFieldOrientedDrive)
        .withPosition(8, 0);
  }

  public void setDrivetrainPoseEstimator(DrivePoseEstimator poseEstimator) {
    this.poseEstimator = poseEstimator;
  }

  public DrivePoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public Mechanism2d getMechVisual() {
    assert (null != poseEstimator);
    return mechVisual;
  }

  public Rotation2d getHeading() {
    assert (null != poseEstimator);
    return poseEstimator.getGyroRotation2d();
  }
  ;

  public abstract void drive(ChassisSpeeds direction);

  @Override
  public void periodic() {
    poseEstimator.update();
    // lockWheels();
  }

  public abstract void setAutoBuilder();

  public void disabledBehavior() {}

  public void toggleFieldOrientedDrive() {
    isFieldOrientedDrive = !isFieldOrientedDrive;
  }

  public void resetOrientation() {
    poseEstimator.resetToPose(
        new Pose2d(
            poseEstimator.getCurrentPose().getTranslation(),
            new Rotation2d(GenericRobot.getAlliance() == Alliance.Blue ? 0 : Math.PI)));
  }

  public void lockWheels() {}

  public Command createDefaultCommand(Controller driver) {
    return new DefaultDriveCommand(
        this,
        () -> driver.getLeftYAxis(),
        () -> driver.getLeftXAxis(),
        () -> driver.getRightXAxis(),
        () -> isFieldOrientedDrive);
  }

  public Command createDefaultTestCommand(Controller driver) {
    return new DefaultDriveCommand(
        this,
        () -> driver.getLeftYAxis(),
        () -> driver.getLeftXAxis(),
        () -> driver.getRightXAxis(),
        () -> isFieldOrientedDrive);
  }

  public boolean hasIssues() {
    return false;
  }

  public void resetEncoders() {}

  public Command generateAutoCommand(Command autoCommand) {
    return autoCommand
        .beforeStarting(
            () -> {
              resetEncoders();
            })
        .until(() -> hasIssues());
  }
}
