// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.crescendo.ShooterSubsystem;

public class RunShooter extends Command {
  private DoubleSupplier shooterSpeed;
  private DoubleSupplier feederSpeed;
  private ShooterSubsystem shooter;
  /** Creates a new Intake. */
  public RunShooter(DoubleSupplier shooterSpeed, DoubleSupplier feederSpeed, ShooterSubsystem shooter) {
    this.shooterSpeed = shooterSpeed;
    this.feederSpeed = feederSpeed;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.stateMachine(shooterSpeed.getAsDouble(), feederSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
