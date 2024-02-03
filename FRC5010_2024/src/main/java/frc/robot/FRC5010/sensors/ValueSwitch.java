package frc.robot.FRC5010.sensors;

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

    public ValueSwitch(double threshold, Supplier<Double> value, double triggerThreshold) {
        this.thresholdSupplier = () -> threshold;
        this.valueSupplier = value;
        this.triggerThreshold = triggerThreshold;
    }
    
    public Boolean get() {
        return (valueSupplier.get() - thresholdSupplier.get()) > triggerThreshold;
    }

    public void setThreshold(Supplier<Double> threshold) {
        thresholdSupplier = threshold;
    }

    public void setThreshold(double threshold) {
        thresholdSupplier = () -> threshold;
    }

    public void setTriggerThreshold(double threshold) {
        triggerThreshold = threshold;
    }


}
