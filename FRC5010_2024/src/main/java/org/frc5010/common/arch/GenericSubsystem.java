// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GenericSubsystem extends SubsystemBase implements WpiHelperInterface {
  protected final WpiNetworkTableValuesHelper values = new WpiNetworkTableValuesHelper();
  protected String logPrefix = getClass().getSimpleName();
  protected Mechanism2d mechanismSimulation;

  /** Creates a new LoggedSubsystem. */
  public GenericSubsystem() {
    WpiNetworkTableValuesHelper.register(this);
  }

  public Mechanism2d getMechSimulation() {
    return mechanismSimulation;
  }

  public void setMechSimulation(Mechanism2d mechSim) {
    mechanismSimulation = mechSim;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    log(logPrefix + ": Initializing sendables.");
    values.initSendables(builder, this.getClass().getSimpleName());
  }
}
