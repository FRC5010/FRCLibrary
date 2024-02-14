// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.IntakeSubsystem;
import frc.robot.crescendo.PivotSubsystem;
import frc.robot.crescendo.ShooterSubsystem;

public class IntakeNote extends Command {
  private PivotSubsystem pivotSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private DoubleSupplier intakeSpeedSupplier;
  private DoubleSupplier feederSpeedSupplier;
  /** Creates a new IntakeNote. */
  public IntakeNote(PivotSubsystem pivot, IntakeSubsystem intake, ShooterSubsystem shooter, DoubleSupplier intakeSpeed, DoubleSupplier feederSpeed) {
    pivotSubsystem = pivot;
    intakeSubsystem = intake;
    shooterSubsystem = shooter;
    intakeSpeedSupplier = intakeSpeed;
    feederSpeedSupplier = feederSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSubsystem.setReference(pivotSubsystem.INTAKE_LEVEL);
    intakeSubsystem.setReference(intakeSpeedSupplier.getAsDouble(), intakeSpeedSupplier.getAsDouble());
    shooterSubsystem.setFeederReference(feederSpeedSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.getBeambreak();
  }
}
