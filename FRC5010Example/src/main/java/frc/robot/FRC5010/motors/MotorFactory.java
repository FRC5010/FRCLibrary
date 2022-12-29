// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.motors;

import frc.robot.FRC5010.motors.function.DriveTrainMotor;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.motors.hardware.NEO550;

/** Add your docs here. */
public class MotorFactory {
    public static MotorController5010 NEO(int port){
        return new NEO(port);
    }
    public static MotorController5010 NEO(int port, int currentLimit){
        return new NEO(port, currentLimit);
    }
    public static MotorController5010 NEO550(int port){
        return new NEO550(port);
    }
    public static MotorController5010 NEO550(int port, int currentLimit){
        return new NEO550(port, currentLimit);
    }
    public static MotorController5010 DriveTrainMotor(MotorController5010 motor){
        return new DriveTrainMotor(motor);
    }
}
