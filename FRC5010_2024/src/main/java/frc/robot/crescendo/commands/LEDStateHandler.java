// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.subsystems.Color;
import frc.robot.FRC5010.subsystems.SegmentedLedSystem;
import frc.robot.crescendo.FeederSubsystem.NoteState;

public class LEDStateHandler extends Command {
  Supplier<NoteState> currentNoteState;
  SegmentedLedSystem ledSubsystem;
  /** Creates a new LEDStateHandler. */
  public LEDStateHandler(SegmentedLedSystem ledSubsystem, Supplier<NoteState> currentNoteState) {
    this.currentNoteState = currentNoteState;
    this.ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    switch (currentNoteState.get()) {
      case Empty:
        ledSubsystem.setLedSegmentColor("Whole", Color.RED);
      break;
      case Holding:
        ledSubsystem.setLedSegmentColor("Whole",  Color.ORANGE);
      break;
      case Loaded:
        ledSubsystem.setLedSegmentColor("Whole", Color.PURPLE);
      break;
      case Shooting:
        ledSubsystem.setLedSegmentColor("Whole", Color.FIFTY_TEN_ORANGE);
      break;
        // TODO: Ready to shoot: Green
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
