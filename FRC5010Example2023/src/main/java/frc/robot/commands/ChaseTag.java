// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.drive.GenericDrivetrain;
import frc.robot.FRC5010.drive.SwerveDrivetrain;


public class ChaseTag extends CommandBase {
  
  private final GenericPID pidTranslation = new GenericPID(1, 0, 0);
  private final GenericPID thetaTranslation = new GenericPID(.25, 0, 0);

  /** Creates a new ChaseTag. */
  private final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(SwerveDrivetrain.kPhysicalMaxSpeedMetersPerSecond, SwerveDrivetrain.kTeleDriveMaxAccelerationUnitsPerSecond); 
  private final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(SwerveDrivetrain.kPhysicalMaxSpeedMetersPerSecond, SwerveDrivetrain.kTeleDriveMaxAccelerationUnitsPerSecond); 
  private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(SwerveDrivetrain.kPhysicalMaxAngularSpeedRadiansPerSecond, SwerveDrivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond); 

  private final ProfiledPIDController xController = new ProfiledPIDController(pidTranslation.getkP(), 0, 0, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(pidTranslation.getkP(), 0, 0, yConstraints);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(thetaTranslation.getkP(), 0, 0, thetaConstraints);


  private final Transform3d tagToGoal = new Transform3d(
    new Translation3d(0.5, 0.0, 0.0),
    new Rotation3d(0.0, 0.0, 0));

    
  private GenericDrivetrain swerveSubsystem; 
  private Supplier<Pose2d> poseProvider; 
  
  public ChaseTag(GenericDrivetrain swerveSubsystem, Supplier<Pose2d> poseProvider) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    thetaController.setTolerance(Units.degreesToRadians(3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(swerveSubsystem);
    
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var robotPose = poseProvider.get(); 

    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
            
      // // Transform the tag's pose to set our goal
      var goalPose = robotPose.transformBy(tagToGoal.inverse()).toPose2d();
      System.out.println(goalPose); 

      // // Drive
      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
      thetaController.setGoal(goalPose.getRotation().getRadians());

      System.out.println("xSpeed: " + xController.calculate(robotPose.getX()) + "\nySpeed: " + yController.calculate(robotPose.getY()) + 
      "\nTheta: " + thetaController.calculate(robotPose2d.getRotation().getRadians())); 
    swerveSubsystem.drive(new ChassisSpeeds(
    xController.calculate(robotPose.getX()) * SwerveDrivetrain.kTeleDriveMaxSpeedMetersPerSecond, 
    yController.calculate(robotPose.getY()) * SwerveDrivetrain.kTeleDriveMaxSpeedMetersPerSecond
    ,thetaController.calculate(robotPose2d.getRotation().getRadians()) * SwerveDrivetrain.kTeleDriveMaxAngularSpeedRadiansPerSecond));
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}