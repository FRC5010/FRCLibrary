// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.motors.function;

import frc.robot.FRC5010.motors.MotorController5010;

/** Add your docs here. */
public class FollowerMotor extends GenericFunctionalMotor {
    public FollowerMotor(MotorController5010 motor, MotorController5010 leader){
        super(motor);
        setFollow(leader);
    }
}
