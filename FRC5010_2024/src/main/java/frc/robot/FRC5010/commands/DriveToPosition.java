// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.constants.GenericCommand;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.subsystems.LedSubsystem;
import frc.robot.chargedup.TargetConstants;

public class DriveToPosition extends GenericCommand {
  private SwerveDrivetrain swerveSubsystem;
  private final GenericPID pidTranslation = new GenericPID(1, 0, 0);
  private final GenericPID thetaTranslation = new GenericPID(.25, 0, 0);

  private final TrapezoidProfile.Constraints xConstraints;
  private final TrapezoidProfile.Constraints yConstraints;
  private final TrapezoidProfile.Constraints thetaConstraints;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  public static enum LCR {
    left, center, right
  }

  Transform2d tagToLeftConeTransfrom = new Transform2d(
      new Translation2d(TargetConstants.tagToRobotXOffset, TargetConstants.tagToConeYOffset),
      Rotation2d.fromDegrees(180));

  Transform2d tagToCubeTransfrom = new Transform2d(new Translation2d(TargetConstants.tagToRobotXOffset, 0),
      Rotation2d.fromDegrees(180));

  Transform2d tagToRightConeTransfrom = new Transform2d(
      new Translation2d(TargetConstants.tagToRobotXOffset, -TargetConstants.tagToConeYOffset),
      Rotation2d.fromDegrees(180));

  private final Pose3d tagToGoal = new Pose3d(
      new Translation3d(1.5, 0.0, 0.0),
      new Rotation3d(0.0, 0.0, 0));

  private Pose2d targetPose;
  private Transform2d targetTransform;

  private Supplier<Pose2d> poseProvider;
  private Supplier<Pose2d> targetPoseProvider;

  private double xSpeed;
  private double ySpeed;
  private double thetaSpeed;

  public DriveToPosition(SwerveDrivetrain swerveSubsystem, Supplier<Pose2d> poseProvider,
      Supplier<Pose2d> targetPoseProvider, LedSubsystem ledSubsystem, LCR relativePosition) {
    xConstraints = new TrapezoidProfile.Constraints(
        swerveSubsystem.getSwerveConstants().getkPhysicalMaxSpeedMetersPerSecond(),
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond());
    yConstraints = new TrapezoidProfile.Constraints(
        swerveSubsystem.getSwerveConstants().getkPhysicalMaxSpeedMetersPerSecond(),
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAccelerationUnitsPerSecond());
    thetaConstraints = new TrapezoidProfile.Constraints(
        swerveSubsystem.getSwerveConstants().getkPhysicalMaxAngularSpeedRadiansPerSecond(),
        swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond());

    xController = new ProfiledPIDController(pidTranslation.getkP(), pidTranslation.getkI(), pidTranslation.getkD(),
        xConstraints);
    yController = new ProfiledPIDController(pidTranslation.getkP(), pidTranslation.getkI(), pidTranslation.getkD(),
        yConstraints);
    thetaController = new ProfiledPIDController(thetaTranslation.getkP(), thetaTranslation.getkI(),
        thetaTranslation.getkD(), thetaConstraints);

    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = (SwerveDrivetrain) swerveSubsystem;
    this.poseProvider = poseProvider;
    this.targetPoseProvider = targetPoseProvider;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.setTolerance(Units.degreesToRadians(3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    switch (relativePosition) {
      case left:
        targetTransform = tagToLeftConeTransfrom;
        break;
      case center:
        targetTransform = tagToCubeTransfrom;
        break;
      case right:
        targetTransform = tagToRightConeTransfrom;
        break;
      default:
        targetTransform = tagToCubeTransfrom;
        break;
    }

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    var robotPose = poseProvider.get();
    targetPose = targetPoseProvider.get().transformBy(targetTransform);

    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    thetaController.setGoal(targetPose.getRotation().getRadians());

    swerveSubsystem.getPoseEstimator().setTargetPoseOnField(targetPose, "Target Pose");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();

    // System.out.println(robotPose2d);

    // System.out.println(robotPose);
    xSpeed = xController.calculate(robotPose2d.getX())
        * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    ySpeed = yController.calculate(robotPose2d.getY())
        * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    thetaSpeed = thetaController.calculate(robotPose2d.getRotation().getRadians())
        * swerveSubsystem.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();

    if (xController.atGoal()) {
      xSpeed = 0;
    }

    if (yController.atGoal()) {
      ySpeed = 0;
    }

    if (thetaController.atGoal()) {
      thetaSpeed = 0;
    }

    // // Transform the tag's pose to set our goal

    // System.out.println(goalPose);
    // SmartDashboard.putNumber("X Speed", xSpeed);
    // SmartDashboard.putNumber("Y Speed", ySpeed);
    // SmartDashboard.putNumber("Theta Speed", thetaSpeed);

    // System.out.println(thetaSpeed);
    // swerveSubsystem.drive(new ChassisSpeeds(-xSpeed, -ySpeed, -thetaSpeed));
    // System.out.println("xSpeed: " + xController.calculate(robotPose.getX()) +
    // "\nySpeed: " + yController.calculate(robotPose.getY()) +
    // "\nTheta: " +
    // thetaController.calculate(robotPose2d.getRotation().getRadians()));

    // swerveSubsystem.drive(new ChassisSpeeds(
    // xController.calculate(robotPose.getX()) *
    // swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond(),
    // yController.calculate(robotPose.getY()) *
    // swerveSubsystem.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond()
    // ,0));

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed,
        swerveSubsystem.getHeading());

    SmartDashboard.putNumber("X Speed", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Y Speed", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Theta Speed", chassisSpeeds.omegaRadiansPerSecond);
    swerveSubsystem.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, swerveSubsystem.getHeading()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xSpeed == 0) && (ySpeed == 0) && (thetaSpeed == 0);
  }
}