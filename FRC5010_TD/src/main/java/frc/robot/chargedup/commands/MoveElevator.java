// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FRC5010.constants.GenericCommand;
import frc.robot.FRC5010.telemetery.WpiDataLogging;
import frc.robot.chargedup.ElevatorLevel;
import frc.robot.chargedup.ElevatorSubsystem;

public class MoveElevator extends GenericCommand {
  ElevatorSubsystem elevator;
  Supplier<ElevatorLevel> elevatorLevel;

  /** Creates a new MoveElevator. */
  public MoveElevator(ElevatorSubsystem elevator, Supplier<ElevatorLevel> elevatorLevel) {
    logPrefix = getName() + "_" + elevatorLevel.get();
    this.elevator = elevator;
    this.elevatorLevel = elevatorLevel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    SmartDashboard.putData(logPrefix, this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    ElevatorSubsystem.setElevatorTargetLevel(elevatorLevel.get());
    WpiDataLogging.log(getName());
    elevator.initializeRunToTarget(elevatorLevel.get().getExtensionPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runExtendToTarget(elevatorLevel.get().getExtensionPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    WpiDataLogging.log(getName() + " ended " + interrupted);

    elevator.stopAndHoldExtend();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isExtendAtTarget();
  }
}
