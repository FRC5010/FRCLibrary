// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.ElevatorSubsystem;

public class PivotElevator extends CommandBase {
  ElevatorSubsystem elevator;
  double targetPosition;
  /** Creates a new PivotElevator. */
  public PivotElevator(ElevatorSubsystem elevator, double targetPosition) {
    this.elevator = elevator;
    this.targetPosition = targetPosition;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPivotPosition(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isPivotAtTarget();
  }
}
