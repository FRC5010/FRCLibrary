// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.FRC5010.constants.GenericMotorConstants;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.constants.GenericSwerveModuleConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.sensors.encoder.CanCoderEncoder;

/** Add your docs here. */
public class MK4SwerveModule extends GenericSwerveModule {

    public static final GenericSwerveModuleConstants MK4_L1 = new GenericSwerveModuleConstants(
            0.10033,
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    public static final GenericSwerveModuleConstants MK4_L2 = new GenericSwerveModuleConstants(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    public static final GenericSwerveModuleConstants MK4_L3 = new GenericSwerveModuleConstants(
            0.10033,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    public static final GenericSwerveModuleConstants MK4_L4 = new GenericSwerveModuleConstants(
            0.10033,
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
    
    private GenericPID pid = new GenericPID(1.0, 0.0, 0.1); 
    private GenericMotorConstants motorConstants = new GenericMotorConstants(0.55641, 0.064889, 0.0025381);
    private GenericSwerveModuleConstants moduleConstants = MK4_L1; 
    
    public MK4SwerveModule(MechanismRoot2d visualRoot, String key, double radOffset, SwervePorts swervePorts) {
                super(visualRoot, key, radOffset);
                super.pid = this.pid;
                super.motorConstants = this.motorConstants;
                super.moduleConstants = this.moduleConstants;
                drive = MotorFactory.NEO(swervePorts.getDrivePort()).invert(moduleConstants.isDrivingInv());
                turn = MotorFactory.NEO(swervePorts.getTurnPort()).invert(moduleConstants.isTurningInv());
                absoluteEncoder = new CanCoderEncoder(swervePorts.getEncoderPort());
                turnEncoder = turn.getMotorEncoder();
                driveEncoder = drive.getMotorEncoder();
        
                // set units drive encoder to meters and meters/sec
                driveEncoder.setPositionConversion(moduleConstants.getkDriveEncoderRot2Meter());
                driveEncoder.setVelocityConversion(moduleConstants.getkDriveEncoderRPM2MeterPerSec());
                // set units turning encoder to radians and radians/sec
                turnEncoder.setPositionConversion(moduleConstants.getkTurningEncoderRot2Rad());
                turnEncoder.setVelocityConversion(moduleConstants.getkTurningEncoderRPM2RadPerSec());
    }
}