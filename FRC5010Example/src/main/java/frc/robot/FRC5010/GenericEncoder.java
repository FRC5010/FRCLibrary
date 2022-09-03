// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010;

/** Add your docs here. */
public interface GenericEncoder {
    double getPosition();
    double getVelocity();
    void reset();
    void setPositionConversion(double conversion);
    void setVelocityConversion(double conversion);
    void setPosition(double position);
    void setRate(double rate);
}
