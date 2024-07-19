// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.common.sensors.Controller;

/**
 * GenericMechanism should be used as the parent class of any mechanism It enforces the use of the
 * functions:
 */
public abstract class GenericMechanism implements WpiHelperInterface, Sendable {
  protected Mechanism2d mechVisual;
  protected ShuffleboardTab shuffleTab;
  protected final WpiNetworkTableValuesHelper values = new WpiNetworkTableValuesHelper();
  protected String logPrefix = getClass().getSimpleName();

  public GenericMechanism(String tabName) {
    this.mechVisual = new Mechanism2d(1, 2);
    shuffleTab = Shuffleboard.getTab(tabName);
    WpiNetworkTableValuesHelper.register(this);
  }

  public GenericMechanism(Mechanism2d robotMechVisual, ShuffleboardTab shuffleTab) {
    this.mechVisual = robotMechVisual;
    this.shuffleTab = shuffleTab;
    WpiNetworkTableValuesHelper.register(this);
  }

  public ShuffleboardTab getDisplayTab() {
    return shuffleTab;
  }

  public void addToTab(String key) {
    values.addToTab(shuffleTab, key);
  }

  public void addToTabList(String list, String key) {
    values.addToTabList(shuffleTab, list, key);
  }

  /**
   * configureButtonBindings should map button/axis controls to commands
   *
   * @param driver - driver joystick
   * @param operator - operator joystick
   */
  public abstract void configureButtonBindings(Controller driver, Controller operator);

  /**
   * setupDefaultCommands should setup the default commands needed by subsystems It could check for
   * Test mode and enable different commands
   */
  public abstract void setupDefaultCommands(Controller driver, Controller operator);

  public void setupTestDefaultCommmands(Controller driver, Controller operator) {}

  /**
   * initRealOrSim should check the real or simulation state of the robot and initialize its code
   * accordingly
   */
  protected abstract void initRealOrSim();

  /** setupPreferences should be implemented in place of using Constants files */
  protected void setupPreferences() {}

  public abstract void initAutoCommands();

  public abstract Command generateAutoCommand(Command autoCommand);

  public void disabledBehavior() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    log(logPrefix + ": Initializing sendables.");
    values.initSendables(builder, this.getClass().getSimpleName());
  }
}
