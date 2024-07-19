// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup.commands;

import org.frc5010.common.arch.GenericCommand;
import frc.robot.chargedup.CheeseStick;

public class CheeseStickCommand extends GenericCommand {
  /** Creates a new CheeseStick. */
  private double position;
  private CheeseStick cheeseStick;

  public CheeseStickCommand(double position, CheeseStick cheeseStick) {
    this.position = position;
    this.cheeseStick = cheeseStick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cheeseStick);
  }

  // Called when the command is initially scheduled.
  @Override
  public void init() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void stop(boolean interrupted) {
    cheeseStick.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cheeseStick.rotateToSetPoint(position);
  }
}
