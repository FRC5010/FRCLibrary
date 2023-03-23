// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LogCommand extends CommandBase {
  /** Creates a new LogCommand. */
  private String message;

  public LogCommand(String message) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.message = message;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log(message);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
