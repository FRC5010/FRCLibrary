// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FRC5010.arch.GenericMechanism;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.sensors.Controller;

/** Add your docs here. */
public class MaxDemoBoard extends GenericMechanism {
  private MotorController5010 kraken1;
  private MotorController5010 neo1;
  private MotorController5010 neo2;
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

  public MaxDemoBoard(Mechanism2d visual, ShuffleboardTab displayTab) {
    super(visual, displayTab);
    
    // Motor Setup
    kraken1 = MotorFactory.KrakenX60(2);
    neo1 = MotorFactory.NEO(5);
    neo2 = MotorFactory.NEO(3);

    // Double Solenoid Setup
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void initAutoCommands() {
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.createAButton().whileTrue(Commands.run(() -> kraken1.set(0.25)).finallyDo(() -> kraken1.set(0.0)));
    driver.createBButton().whileTrue(Commands.run(() -> neo1.set(0.25)).finallyDo(() -> neo1.set(0.0)));
    driver.createYButton().whileTrue(Commands.run(() -> neo2.set(0.25)).finallyDo(() -> neo2.set(0.0)));
    driver.createXButton().whileTrue(Commands.runOnce(() -> m_doubleSolenoid.toggle()));
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
  }

  @Override
  protected void initRealOrSim() {
  }

  @Override
  public Command generateAutoCommand(Command autoCommand) {
    return autoCommand;
  }
}
