// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.arch.GenericCommand;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.chargedup.IntakeSubsystem;

public class AutoIntake extends GenericCommand {
  SwerveDrivetrain drive;
  VisionSystem visionSystem;
  double lastAngleY;
  double noNoteCounter = 0;
  private final double NO_NOTE_THRESHOLD = 60;

  private final String X_SPEED = "X Speed";
  private final String Y_SPEED = "Y Speed";
  private final String ANGLE_SPEED = "Angle Speed";

  private final double DEF_X = -0.025;
  private final double DEF_Y = 0.005;
  private final double DEF_ANGLE = -0.015;


  /** Creates a new AutoIntake. */
  public AutoIntake(SwerveDrivetrain drive, VisionSystem vision) {
    this.drive = drive;
    this.visionSystem = vision;

    SmartDashboard.putNumber(X_SPEED, DEF_X);
    SmartDashboard.putNumber(Y_SPEED, DEF_Y);
    SmartDashboard.putNumber(ANGLE_SPEED, DEF_ANGLE);

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    lastAngleY = visionSystem.getAngleY();
    noNoteCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngleY = visionSystem.getAngleY();
    double currentAngleX = visionSystem.getAngleX();
    



    if (currentAngleY > lastAngleY + 5 || !visionSystem.isValidTarget()) {
      currentAngleX = 0;
      currentAngleY = 0;
      noNoteCounter++;
    } else {
      lastAngleY = currentAngleY;
      noNoteCounter = 0;
    }

    SmartDashboard.putNumber("No Note Counter", noNoteCounter);

    double distanceFactor = Math.abs(currentAngleY / 10.0 + 0.0001);
    distanceFactor = Math.min(distanceFactor, 1.0);
    

    double xSpeed = currentAngleY * SmartDashboard.getNumber(X_SPEED, DEF_X);
    double ySpeed = currentAngleX * SmartDashboard.getNumber(Y_SPEED, DEF_Y) / distanceFactor;
    double turnSpeed = currentAngleX * SmartDashboard.getNumber(ANGLE_SPEED, DEF_ANGLE) * distanceFactor;
    xSpeed = 1.10 * Math.sin(-Units.degreesToRadians(currentAngleY));
    ySpeed = 0.35 * Math.sin(Units.degreesToRadians(currentAngleX));
    turnSpeed = 0.18 * Math.sin(-Units.degreesToRadians(currentAngleX));

    // limit power
    xSpeed = xSpeed * drive.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    ySpeed = ySpeed * drive.getSwerveConstants().getkTeleDriveMaxSpeedMetersPerSecond();
    turnSpeed = turnSpeed * drive.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();

    //
    // System.out.println(swerveDrive.getGyroRate());
    double gyroRate = Units.degreesToRadians(drive.getGyroRate()) * 0.01;

   
    drive.drive(new ChassisSpeeds(xSpeed, ySpeed, turnSpeed));

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    SmartDashboard.putNumber("No Note Counter", 0.0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noNoteCounter > NO_NOTE_THRESHOLD;
  }
}
