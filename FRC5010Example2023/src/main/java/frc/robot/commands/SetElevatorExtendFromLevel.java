// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.ElevatorLevel;
import frc.robot.chargedup.ElevatorSubsystem;

public class SetElevatorExtendFromLevel extends CommandBase {
  ElevatorSubsystem elevator;
  ElevatorLevel targetLevel;
  /** Creates a new SetElevatorLevel. */
  public SetElevatorExtendFromLevel(ElevatorSubsystem elevator, ElevatorLevel targetLevel) {
    this.elevator = elevator;
    this.targetLevel = targetLevel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  public SetElevatorExtendFromLevel(ElevatorSubsystem elevator) {
    this.elevator = elevator;
    this.targetLevel = elevator.getElevatorLevel();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setExtendPosition(targetLevel.getExtensionPosition());
    elevator.setElevatorLevel(targetLevel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isExtendAtTarget();
  }
}
