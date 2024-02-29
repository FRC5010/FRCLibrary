// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.FRC5010.arch.GenericSubsystem;

public class PowerDistribution5010
 extends GenericSubsystem {
  private PowerDistribution powerDistribution = new PowerDistribution();
  private final String TOTAL_CURRENT = "Total Current";

  /** Creates a new PowerDistribution. */
  public PowerDistribution5010() {
    values.declare(TOTAL_CURRENT, 0.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    values.set(TOTAL_CURRENT, powerDistribution.getTotalCurrent());
  }
}
