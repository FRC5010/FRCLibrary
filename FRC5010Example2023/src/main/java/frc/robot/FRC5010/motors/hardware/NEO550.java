// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.motors.hardware;

import frc.robot.FRC5010.motors.MotorConstants;

/** 
 * NEO550 extends CANSparkMax and applies code specific to a NEO550
 */
public class NEO550 extends GenericRevBrushlessMotor {
    public NEO550(int port){
        super(port, MotorConstants.CurrentLimits.Neo550);
    }

    public NEO550(int port, int currentLimit){
        super(port, currentLimit);
    }
}
