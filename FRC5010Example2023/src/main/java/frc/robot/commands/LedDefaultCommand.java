// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.subsystems.LedSubsystem;
import frc.robot.chargedup.ElevatorSubsystem;
import frc.robot.chargedup.IntakeSubsystem;

public class LedDefaultCommand extends CommandBase {
  /** Creates a new LedDefaultCommand. */
  LedSubsystem ledSubsystem;
  IntakeSubsystem intakeSubsystem;
  ElevatorSubsystem elevator;
  private int currDelay = 200;
  private boolean lastState;

  public LedDefaultCommand(LedSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubsystem = ledSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elevator = elevator;

    lastState = !intakeSubsystem.isIntakeCone();
    addRequirements(ledSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lastState != ledSubsystem.getLedConeMode()) {
      if (ledSubsystem.getLedConeMode()) {
        // ledSubsystem.setOrbit(210, 255, 0, 0, 0, 0, .1);
        if (elevator.isExtendAtTarget()) {
          ledSubsystem.setSolidColor(210, 255, 0);
        } else {
          ledSubsystem.setBlink(210, 255, 0, 200);
        }
        // (210, 255, 0);
        // ledSubsystem.speed(1);
      } else {
        // ledSubsystem.setOrbit(100, 0, 255, 0, 0, 0, .1);
        if (elevator.isExtendAtTarget()) {
          ledSubsystem.setSolidColor(100, 0, 255);
        } else {
          ledSubsystem.setBlink(100, 0, 255, 200);
        }
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
