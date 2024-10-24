// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FRC5010.arch.GenericMechanism;


/** Add your docs here. */
public class MaxDemoBoard extends GenericRobot {
  private MotorController5010 kraken1;
  private MotorController5010 neo1;
  private MotorController5010 neo2;
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

  public MaxDemoBoard() {
    super();
    
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


  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.createAButton().whileTrue(Commands.run(() -> kraken1.set(0.25)).finallyDo(() -> kraken1.set(0.0)));
    driver.createBButton().whileTrue(Commands.run(() -> neo1.set(0.25)).finallyDo(() -> neo1.set(0.0)));
    driver.createYButton().whileTrue(Commands.run(() -> neo2.set(0.25)).finallyDo(() -> neo2.set(0.0)));
    driver.createXButton().whileTrue(Commands.runOnce(() -> m_doubleSolenoid.toggle()));
  }


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
