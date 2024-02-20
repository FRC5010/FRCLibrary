// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.FeederSubsystem;

public class RunFeeder extends Command {
  private FeederSubsystem feederSubsystem;
  private DoubleSupplier speed;
  /** Creates a new RunFeeder. */
  public RunFeeder(FeederSubsystem feederSubsystem, DoubleSupplier speed) {
    this.feederSubsystem = feederSubsystem;
    this.speed = speed;

    addRequirements(feederSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsystem.setFeederSpeed(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
