// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.crescendo.PivotSubsystem;

public class RunPivot extends Command {
  PivotSubsystem pivotSubsystem;
  Supplier<Double> speed; 
  /** Creates a new runPivot. */
  public RunPivot(Supplier<Double> speed, PivotSubsystem pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    pivotSubsystem = pivot;
    this.speed = speed;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotSubsystem.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsystem.setSpeed(speed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
