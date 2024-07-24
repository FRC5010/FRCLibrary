// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveDrivetrain;
import org.frc5010.common.vision.AprilTags;

public class DriveToTrajectory extends Command {

  public static enum LCR {
    left,
    center,
    right
  }

  private SwerveDrivetrain swerveDrivetrain;

  private PathPlannerTrajectory trajectory;
  //  private PathPlannerState currentSetpoint;

  private PIDController translationController, thetaController;
  private PPHolonomicDriveController driveController;

  private Pose2d currentPose;
  private Pose2d targetPose;
  private Pose2d kPositionTolerance = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3));

  private Timer timer;
  private PathConstraints constraints;

  // Heading based on alliance color
  private Rotation2d desiredHeading;

  public DriveToTrajectory(
      SwerveDrivetrain swerveDrivetrain,
      LCR relativePosition,
      SwerveConstants constants,
      double xOffset,
      double yOffset) {
    this.swerveDrivetrain = swerveDrivetrain;

    // constraints = new PathConstraints(constants.getkTeleDriveMaxSpeedMetersPerSecond(),
    //     constants.getkTeleDriveMaxAccelerationUnitsPerSecond());
    timer = new Timer();

    translationController = new PIDController(1, 0, 0);
    thetaController = new PIDController(0.25, 0, 0);

    // desiredHeading = DriverStation.getAlliance() == Alliance.Blue ? new Rotation2d(180) : new
    // Rotation2d(0);
    xOffset = desiredHeading.getDegrees() == 180 ? xOffset : -xOffset;

    switch (relativePosition) {
      case left:
        yOffset = -yOffset;
        break;
      case right:
        break;
      default:
        yOffset = 0;
        break;
    }

    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    int closestTagID = swerveDrivetrain.getPoseEstimator().getClosestTagToRobot();

    currentPose = swerveDrivetrain.getPoseEstimator().getCurrentPose();
    targetPose = AprilTags.aprilTagFieldLayout.getTagPose(closestTagID).get().toPose2d();

    // PathPoint startPoint = new PathPoint(new Translation2d(currentPose.getX(),
    // currentPose.getY()),
    //     swerveDrivetrain.getPoseEstimator().getGyroRotation2d());
    // PathPoint endPoint = new PathPoint(new Translation2d(targetPose.getX() + xOffset,
    // targetPose.getY() + yOffset),
    //     desiredHeading);

    // trajectory = PathPlanner.generatePath(constraints, startPoint, endPoint);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // driveController = new PPHolonomicDriveController(translationController,
    // translationController, thetaController);
    // driveController.setTolerance(kPositionTolerance);

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //    currentSetpoint = (PathPlannerState) trajectory.sample(timer.get());
    currentPose = swerveDrivetrain.getPoseEstimator().getCurrentPose();

    //  this.swerveDrivetrain.drive(this.driveController.calculate(currentPose, currentSetpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    driveController.setEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // (trajectory.getTotalTimeSeconds() < timer.get() &&
    // driveController.atReference());
  }
}
