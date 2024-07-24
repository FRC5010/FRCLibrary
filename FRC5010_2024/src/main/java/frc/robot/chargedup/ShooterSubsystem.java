// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.motors.MotorController5010;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  MotorController5010 shooterMotor;
  MotorController5010 feederMotor;
  public ShooterSubsystem(MotorController5010 shooterMotor, MotorController5010 feederMotor) {
    this.feederMotor = feederMotor;
    this.shooterMotor = shooterMotor;
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  public void stop() {
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
