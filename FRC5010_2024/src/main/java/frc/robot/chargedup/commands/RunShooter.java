// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chargedup.ShooterSubsystem;

public class RunShooter extends Command {
  /** Creates a new RunShooter. */
  ShooterSubsystem shooter;
  DoubleSupplier speed;
  public RunShooter(ShooterSubsystem shooter, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.speed = speed;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
