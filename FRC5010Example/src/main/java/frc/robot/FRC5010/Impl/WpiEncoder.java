// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Impl;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.FRC5010.GenericEncoder;

/** Add your docs here. */
public class WpiEncoder implements GenericEncoder {
    Encoder encoder;

    public WpiEncoder(Encoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public double getPosition() {
        return encoder.getDistance();
    }

    @Override
    public double getVelocity() {
        return encoder.getRate();
    }

    @Override
    public void reset() {
        encoder.reset();
    }

    @Override
    public void setPosition(double position) {
    }

    @Override
    public void setRate(double rate) {
    }

    @Override
    public void setPositionConversion(double conversion) {
        encoder.setDistancePerPulse(conversion);
    }

    @Override
    public void setVelocityConversion(double conversion) {
        encoder.setDistancePerPulse(conversion);
    }
}
