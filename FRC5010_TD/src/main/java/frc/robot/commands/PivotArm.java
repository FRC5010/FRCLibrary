// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.telemetery.WpiDataLogging;
import frc.robot.chargedup.ArmLevel;
import frc.robot.chargedup.PivotSubsystem;

public class PivotArm extends CommandBase {
  PivotSubsystem pivotSubsystem;
  ArmLevel armLevel;

  /** Creates a new PivotElevator. */
  public PivotArm(PivotSubsystem pivot, ArmLevel elevatorLevel) {
    this.pivotSubsystem = pivot;
    this.armLevel = elevatorLevel;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    WpiDataLogging.log(getName() + " " + armLevel.name());
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    pivotSubsystem.runPivotToTarget(armLevel.getPivotPosition());
    System.out.println("Running to target " + armLevel.name() + " " + armLevel.getPivotPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    WpiDataLogging.log(getName() + " ended " + interrupted);
    System.out.println("Ended PivotArm");
    pivotSubsystem.stopAndHoldPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotSubsystem.isPivotAtTarget()
        || (pivotSubsystem.isPivotMaxPosition() && pivotSubsystem.getVelocity() > 0.1)
        || (pivotSubsystem.isPivotMinPosition() && pivotSubsystem.getVelocity() < -0.1);
  }
}
