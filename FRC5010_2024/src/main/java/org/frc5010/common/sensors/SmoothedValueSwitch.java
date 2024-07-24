// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors;

import edu.wpi.first.math.filter.LinearFilter;
import java.util.function.Supplier;

/** Add your docs here. */
public class SmoothedValueSwitch extends ValueSwitch {
  private LinearFilter valueFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  public SmoothedValueSwitch(
      Supplier<Double> threshold, Supplier<Double> value, double triggerThreshold) {
    super(threshold, value, triggerThreshold);
  }

  public SmoothedValueSwitch(double threshold, Supplier<Double> value, double triggerThreshold) {
    super(threshold, value, triggerThreshold);
  }

  @Override
  public Boolean get() {
    return (valueFilter.calculate(valueSupplier.get()) - thresholdSupplier.get())
        > triggerThreshold;
  }

  public void updateValue() {
    valueFilter.calculate(valueSupplier.get());
  }
}
