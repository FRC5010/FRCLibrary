// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.subsystems.LedSubsystem;
import frc.robot.chargedup.IntakeSubsystem;

public class LedDefaultCommand extends CommandBase {
  /** Creates a new LedDefaultCommand. */
  LedSubsystem ledSubsystem;
  IntakeSubsystem intakeSubsystem;
  public LedDefaultCommand(LedSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubsystem = ledSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(ledSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeSubsystem.isIntakeCone()){
      ledSubsystem.setSolidColor(255, 255, 0);
      // ledSubsystem.speed(1);
    }else{
      ledSubsystem.setSolidColor(255, 0, 255);
      // ledSubsystem.speed(2);
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
