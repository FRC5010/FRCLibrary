package frc.robot.FRC5010.sensors;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ValueSwitch {
    Supplier<Double> thresholdSupplier;
    Supplier<Double> valueSupplier;
    double triggerThreshold;

    public ValueSwitch(Supplier<Double> threshold, Supplier<Double> value, double triggerThreshold) {
        this.thresholdSupplier = threshold;
        this.valueSupplier = value;
        this.triggerThreshold = triggerThreshold;
    }
    

    public Boolean get() {
        return (valueSupplier.get() - thresholdSupplier.get()) > triggerThreshold;
    }


}
