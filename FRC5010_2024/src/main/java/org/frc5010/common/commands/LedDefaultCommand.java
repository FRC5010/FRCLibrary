// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.common.subsystems.LedSubsystem;

public class LedDefaultCommand extends Command {
  /** Creates a new LedDefaultCommand. */
  LedSubsystem ledSubsystem;

  private int currDelay = 200;
  private boolean lastState;

  public LedDefaultCommand(LedSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubsystem = ledSubsystem;
    lastState = false;
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lastState != ledSubsystem.getLedConeMode()) {
      if (ledSubsystem.getLedConeMode()) {
        // ledSubsystem.setOrbit(210, 255, 0, 0, 0, 0, .1);
        ledSubsystem.setBlink(210, 255, 0, 200);
        // (210, 255, 0);
        // ledSubsystem.speed(1);
      } else {
        // ledSubsystem.setOrbit(100, 0, 255, 0, 0, 0, .1);
        ledSubsystem.setBlink(100, 0, 255, 200);
        // ledSubsystem.setSolidColor(100, 0, 255);
        // ledSubsystem.speed(2);

      }
      lastState = ledSubsystem.getLedConeMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSubsystem.setRainbow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
