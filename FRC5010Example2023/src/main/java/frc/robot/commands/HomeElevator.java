// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.ElevatorSubsystem;
import frc.robot.chargedup.PivotSubsystem;

public class HomeElevator extends CommandBase {
  ElevatorSubsystem elevatorSubsystem;
  PivotSubsystem pivot;

  /** Creates a new HomeElevator. */
  public HomeElevator(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivot) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.pivot = pivot;
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // DataLogManager.log(getName());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.elevatorSubsystem.runExtendToTarget(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // DataLogManager.log(getName() + " ended " + interrupted);
    this.elevatorSubsystem.stopAndHoldExtend();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.elevatorSubsystem.isElevatorIn();
  }
}
