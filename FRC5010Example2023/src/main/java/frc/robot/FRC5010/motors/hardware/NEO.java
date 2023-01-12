// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.motors.hardware;

import frc.robot.FRC5010.motors.MotorConstants;

/** Add your docs here. */
public class NEO extends GenericRevBrushlessMotor {
    public NEO(int port){
        super(port, MotorConstants.CurrentLimits.Neo);
    }
    public NEO(int port, int currentLimit){
        super(port, currentLimit);
    }
}
