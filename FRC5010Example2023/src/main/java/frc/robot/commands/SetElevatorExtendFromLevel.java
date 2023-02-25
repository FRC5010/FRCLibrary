// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.ElevatorLevel;
import frc.robot.chargedup.ElevatorSubsystem;

public class SetElevatorExtendFromLevel extends CommandBase {
  ElevatorSubsystem elevator;
  ElevatorLevel targetLevel;
  /** Creates a new SetElevatorLevel. */
  public SetElevatorExtendFromLevel(ElevatorSubsystem elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  public SetElevatorExtendFromLevel(ElevatorSubsystem elevator, ElevatorLevel level) {
    this.elevator = elevator;
    this.targetLevel = level; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (null == targetLevel){
      elevator.setExtendPosition(elevator.getElevatorLevel().getExtensionPosition());
    } else {
      elevator.setElevatorLevel(targetLevel);
    }
    SmartDashboard.putNumber("Elevator Level: ", elevator.getElevatorLevel().getExtensionPosition()); 
    SmartDashboard.putNumber("Curr Pos: ", elevator.getExtendPosition()); 
      // elevator.setElevatorLevel(elevator.getElevatorLevel());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(interrupted); 
    elevator.extendPow(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isExtendAtTarget();
  }
}
