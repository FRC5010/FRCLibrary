// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.motors;

import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;

/** Add your docs here. */
public interface MotorController5010 extends MotorController {
    /** Sets up the same motor hardware and current limit */
    MotorController5010 duplicate(int port);
    MotorController5010 setSlewRate(double rate);
    MotorController5010 setFollow(MotorController5010 motor);
    MotorController5010 setFollow(MotorController5010 motor, boolean inverted); 
    MotorController5010 invert(boolean inverted);
    MotorController5010 setCurrentLimit(int limit);
    GenericEncoder getMotorEncoder();
    GenericEncoder getMotorEncoder(Type sensorType, int countsPerRev);
    MotorController getMotor();
    void factoryDefault();
}
