// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.ElevatorLevel;
import frc.robot.chargedup.ElevatorSubsystem;

public class SetElevatorPivotFromLevel extends CommandBase {

  ElevatorSubsystem elevator;
  ElevatorLevel level;

  /** Creates a new SetElevatorPivotFromLevel. */
  public SetElevatorPivotFromLevel(ElevatorSubsystem elevator, ElevatorLevel level) {
    this.elevator = elevator;
    this.level = level;
    addRequirements(elevator);
  }

  public SetElevatorPivotFromLevel(ElevatorSubsystem elevator) {
    this.elevator = elevator;
    this.level = elevator.getElevatorLevel();
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setElevatorLevel(level);
    elevator.setPivotPosition(level.getPivotPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.pivotPow(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isPivotAtTarget();
  }
}
