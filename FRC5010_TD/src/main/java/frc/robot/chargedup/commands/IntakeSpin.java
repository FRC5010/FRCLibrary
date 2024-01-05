// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.commands;

import java.util.function.Supplier;

import frc.robot.FRC5010.constants.GenericCommand;
import frc.robot.chargedup.IntakeSubsystem;

public class IntakeSpin extends GenericCommand {

  private Supplier<Double> spinVelocity;
  private IntakeSubsystem intakeSubsystem;

  /** Creates a new IntakeSpin. */
  public IntakeSpin(IntakeSubsystem intakeSubsystem, Supplier<Double> spinVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.spinVelocity = spinVelocity;
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = (this.spinVelocity.get());
    // this.intakeSubsystem.setVelocity(velocity);
    this.intakeSubsystem.setMotor(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    this.intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
