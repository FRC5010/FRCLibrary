// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.common.subsystems.LedSubsystem;

public class LedColor extends Command {
  /** Creates a new LedColor. */
  private int red, green, blue;

  private LedSubsystem ledSubsystem;

  public LedColor(int red, int green, int blue, LedSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.red = red;
    this.green = green;
    this.blue = blue;
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.setSolidColor(red, green, blue);
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
    return false;
  }
}
