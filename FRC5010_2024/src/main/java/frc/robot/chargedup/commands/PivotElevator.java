// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.commands;

import frc.robot.FRC5010.arch.GenericCommand;
import frc.robot.FRC5010.telemetery.WpiDataLogging;
import frc.robot.chargedup.ElevatorLevel;
import frc.robot.chargedup.ElevatorSubsystem;
import frc.robot.chargedup.PivotSubsystem;

public class PivotElevator extends GenericCommand {
  PivotSubsystem pivotSubsystem;
  ElevatorLevel elevatorLevel;

  /** Creates a new PivotElevator. */
  public PivotElevator(PivotSubsystem pivot, ElevatorLevel elevatorLevel) {
    this.pivotSubsystem = pivot;
    this.elevatorLevel = elevatorLevel;
    values.declare("kP", 0.1);
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
    ElevatorSubsystem.setElevatorTargetLevel(elevatorLevel);
    pivotSubsystem.initializeRunToTarget(elevatorLevel.getPivotPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    pivotSubsystem.runPivotToTarget(elevatorLevel.getPivotPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    pivotSubsystem.stopAndHoldPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double goalPos = pivotSubsystem.getPivotTarget();
    double currPos = elevatorLevel.getPivotPosition();
    return pivotSubsystem.isPivotAtTarget()
        || (pivotSubsystem.isPivotMaxPosition() && pivotSubsystem.getVelocity() > 0)
        || (pivotSubsystem.isPivotMinHallEffect() && pivotSubsystem.getVelocity() < 0);
  }
}
