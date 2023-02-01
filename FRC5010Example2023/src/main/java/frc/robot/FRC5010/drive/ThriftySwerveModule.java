// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.FRC5010.constants.GenericSwerveConstants;
import frc.robot.FRC5010.constants.GenericSwerveModuleConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.sensors.AnalogInput5010;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;

/** Add your docs here. */
public class ThriftySwerveModule extends GenericSwerveModule{

    private ProfiledPIDController turningController;

    private MotorController5010 drive, turn;
    private GenericEncoder turnEncoder, driveEncoder, absoluteEncoder;

    private double radOffset;

    // SWERVE CONSTANTS
    
    // neo 550 sysid values
    public static final double kSC = 0.55641;
    public static final double kVC = 0.064889;
    public static final double kAC = 0.0025381;

    public static double kS = kSC / 12;
    public static double kV = kVC / 60 / 1 / (12 - kS);
    public static double kA = kAC / 60 / 1 / (12 - kS);

    // pid values for the neo 550
    public static final double kPTurning = 0.052037 * 4;
    public static final double kITurning = 0;
    public static final double kDTurning = 0;

    // physical values
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kDriveMotorGearRatio = 1/5.25;
    public static final double kTurningMotorGearRatio = 1/((5.33) * 10.5); // not 12:1 but 10.5 for gearbox, ultraplanetaries are not nominal
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    // conversions
    public static final double maxAbsEncoderVolts = 4.815;
    public static final double voltsToDegrees = (360/maxAbsEncoderVolts);
    public static final double voltsToRadians = (2*Math.PI/maxAbsEncoderVolts);
    
    // limits
    public static final int neoCurrentLimit = 60;
    public static final int neo550CurrentLimit = 30;
    

    public ThriftySwerveModule(MechanismRoot2d visualRoot, String key, double radOffset, SwervePorts swervePorts, GenericSwerveModuleConstants swerveConstants, GenericSwerveConstants swerveConstants2) {
        super(visualRoot, key, radOffset, swerveConstants2);
                super.pid = this.pid;
                super.motorConstants = this.motorConstants;
                super.moduleConstants = this.moduleConstants;
                drive = MotorFactory.NEO(swervePorts.getDrivePort()).invert(moduleConstants.isDrivingInv());
                turn = MotorFactory.NEO(swervePorts.getTurnPort()).invert(moduleConstants.isTurningInv());
                absoluteEncoder = new AnalogInput5010(swervePorts.getEncoderPort());
                turnEncoder = turn.getMotorEncoder();
                driveEncoder = drive.getMotorEncoder();
        
                // set units drive encoder to meters and meters/sec
                driveEncoder.setPositionConversion(moduleConstants.getkDriveEncoderRot2Meter());
                driveEncoder.setVelocityConversion(moduleConstants.getkDriveEncoderRPM2MeterPerSec());
                // set units turning encoder to radians and radians/sec
                turnEncoder.setPositionConversion(moduleConstants.getkTurningEncoderRot2Rad());
                turnEncoder.setVelocityConversion(moduleConstants.getkTurningEncoderRPM2RadPerSec());

                setupSwerveEncoders();
    }
}
