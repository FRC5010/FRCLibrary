// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.subsystems.LedSubsystem;

public class LedBlink extends Command {
  /** Creates a new LedColor. */
  private int red,green,blue;
  private long delay;
  private LedSubsystem ledSubsystem;
  public LedBlink(int red, int green, int blue, long delay, LedSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.red = red;
    this.green = green;
    this.blue = blue;
    this.delay = delay;
    this.ledSubsystem = ledSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.setBlink(red, green, blue, delay);
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
    return true;
  }
}
