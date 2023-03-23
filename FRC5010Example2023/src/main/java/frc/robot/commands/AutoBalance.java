// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.drive.GenericDrivetrain;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */

  private GenericDrivetrain drivetrain;

  private int offBalanceThreshold = 7;
  private int onBalanceThreshold = 2;
  private GenericGyro pigeon;
  // private AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private Supplier<Boolean> fieldOrientedDrive;

  public AutoBalance(GenericDrivetrain drivetrain, Supplier<Boolean> fieldOrientedDrive, GenericGyro pigeon) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.fieldOrientedDrive = fieldOrientedDrive;
    this.pigeon = pigeon;

    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log(getName());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAxisRate = 0;
    double yAxisRate = 0;

    boolean autoBalanceXMode = false;
    boolean autoBalanceYMode = false;

    double pitchAngleDegrees = pigeon.getAngleY();
    double rollAngleDegrees = pigeon.getAngleX();

    if (!autoBalanceXMode &&
        (Math.abs(pitchAngleDegrees) >= Math.abs(offBalanceThreshold))) {
      autoBalanceXMode = true;
    } else if (autoBalanceXMode &&
        (Math.abs(pitchAngleDegrees) <= Math.abs(onBalanceThreshold))) {
      autoBalanceXMode = false;
    }

    if (!autoBalanceYMode &&
        (Math.abs(rollAngleDegrees) >= Math.abs(offBalanceThreshold))) {
      autoBalanceYMode = true;
    } else if (autoBalanceYMode &&
        (Math.abs(rollAngleDegrees) <= Math.abs(onBalanceThreshold))) {
      autoBalanceYMode = false;
    }

    // Control drive system automatically,
    // driving in reverse direction of pitch/roll angle,
    // with a magnitude based upon the angle

    if (autoBalanceXMode) {
      double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      xAxisRate = Math.sin(pitchAngleRadians) * -2; // Should be max speed constants add it in later
    }
    if (autoBalanceYMode) {
      double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
      yAxisRate = Math.sin(rollAngleRadians) * 2; // Should be max speed constants add it in later
    }

    SmartDashboard.putNumber("X-Axis Rate", xAxisRate);
    SmartDashboard.putNumber("Y-Axis Rate", yAxisRate);

    SmartDashboard.putNumber("Roll Angle Degrees", rollAngleDegrees);
    SmartDashboard.putNumber("Pitch Angle Degrees", pitchAngleDegrees);

    drivetrain.drive(new ChassisSpeeds(xAxisRate, yAxisRate, 0));

    if (!autoBalanceXMode && !autoBalanceYMode) {
      drivetrain.lockWheels();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log(getName() + " ended " + interrupted);
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    drivetrain.lockWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
