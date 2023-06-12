// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.telemetery.WpiDataLogging;
import frc.robot.chargedup.ElevatorLevel;
import frc.robot.chargedup.ElevatorSubsystem;

public class MoveElevator extends CommandBase {
  ElevatorSubsystem elevator;
  Supplier<ElevatorLevel> elevatorLevel;

  /** Creates a new MoveElevator. */
  public MoveElevator(ElevatorSubsystem elevator, Supplier<ElevatorLevel> elevatorLevel) {
    this.elevator = elevator;
    this.elevatorLevel = elevatorLevel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    WpiDataLogging.log(getName() + " " + elevatorLevel.get().name());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runExtendToTarget(elevatorLevel.get().getExtensionPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (interrupted){
    // elevator.stopExtend();
    // } else {
    // elevator.stopAndHoldExtend();
    // }
    WpiDataLogging.log(getName() + " ended " + interrupted);

    elevator.stopAndHoldExtend();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isExtendAtTarget();
  }
}
