// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

public class GenericCommand extends Command implements WpiHelperInterface {
  protected String logPrefix = getName();
  protected final WpiNetworkTableValuesHelper values = new WpiNetworkTableValuesHelper();

  /** Creates a new LogCommand. */
  public GenericCommand(String logPrefix) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.logPrefix = logPrefix;
    WpiNetworkTableValuesHelper.register(this);
  }

  public GenericCommand() {}

  // Called when the command is initially scheduled.
  @Override
  public final void initialize() {
    log(logPrefix + ": Initializing");
    init();
  }

  public void init() {}

  // Called once the command ends or is interrupted.
  @Override
  public final void end(boolean interrupted) {
    stop(interrupted);
    log(logPrefix + ": " + (interrupted ? "Interrupted: " : "Ended: "));
  }

  public void stop(boolean interrupted) {}

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    log(logPrefix + ": Initializing sendables.");
    values.initSendables(builder, logPrefix);
  }
}
