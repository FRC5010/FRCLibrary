// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Impl;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.FRC5010.GenericEncoder;

/** Add your docs here. */
public class SimulatedEncoder implements GenericEncoder {
    Encoder encoder;
    EncoderSim encoderSim;

    public SimulatedEncoder(Encoder encoder) {
        this.encoder = encoder;
        init();
    }

    public SimulatedEncoder(int port1, int port2) {
        this.encoder = new Encoder(port1, port2);
        init();
    }

    private void init() {
        encoderSim = new EncoderSim(encoder); 
    }

    @Override
    public double getPosition() {
        return encoderSim.getDistance();
    }

    @Override
    public double getVelocity() {
        return encoderSim.getRate();
    }

    @Override
    public void reset() {
        encoderSim.resetData();
    }
    @Override
    public void setPosition(double position) {
       encoderSim.setDistance(position);
    }
    @Override
    public void setRate(double rate) {
        encoderSim.setRate(rate);
    }
    @Override
    public void setPositionConversion(double conversion) {
        encoder.setDistancePerPulse(conversion);
    }
    @Override
    public void setVelocityConversion(double conversion) {     
    }
}
