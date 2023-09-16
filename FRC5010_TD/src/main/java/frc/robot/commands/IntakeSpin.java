// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.telemetery.WpiDataLogging;
import frc.robot.chargedup.IntakeSubsystem;

public class IntakeSpin extends CommandBase {

  private Supplier<Double> spinVelocity;
  private IntakeSubsystem intakeSubsystem;

  /** Creates a new IntakeSpin. */
  public IntakeSpin(IntakeSubsystem intakeSubsystem, Supplier<Double> spinVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.spinVelocity = spinVelocity;
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    WpiDataLogging.log(getName());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = (this.spinVelocity.get());
    if (velocity > 0) {
      velocity *= .25;
    }
    // this.intakeSubsystem.setVelocity(velocity);
    this.intakeSubsystem.setMotor(velocity);
    WpiDataLogging.log("Intake Current:" + intakeSubsystem.getMotorCurrent()); // Message for Logging to see Intake
                                                                               // Current Spike
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.stopIntake();

    WpiDataLogging.log(getName() + " ended. ");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
