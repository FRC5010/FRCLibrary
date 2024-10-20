// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.function.DoubleSupplier;

import org.frc5010.common.arch.GenericCommand;
import frc.robot.crescendo.ClimbSubsystem;

public class RunClimb extends GenericCommand {


  DoubleSupplier leftSpeed;
  DoubleSupplier rightSpeed;

  ClimbSubsystem climbSubsystem;
  /** Creates a new JoystickClimb. */
  public RunClimb(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, ClimbSubsystem climbSubsystem) {

    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    this.climbSubsystem = climbSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    climbSubsystem.setLeftMotorSpeed(0);
    climbSubsystem.setRightMotorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.climbStateMachine(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    climbSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
