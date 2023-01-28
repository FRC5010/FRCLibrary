// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.FRC5010.constants.GenericMotorConstants;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.constants.GenericSwerveModuleConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.sensors.AnalogInput5010;

/** Add your docs here. */
public class ThriftySwerveModule extends GenericSwerveModule{

    private GenericPID pid = new GenericPID(0.052037 * 4, 0, 0); 
    private GenericMotorConstants motorConstants = new GenericMotorConstants(0.55641, 0.064889, 0.0025381);
    private GenericSwerveModuleConstants moduleConstants = new GenericSwerveModuleConstants(Units.inchesToMeters(3), 1/5.25, true, 1/((5.33) * 10.5), true, true); 
    
    
    public ThriftySwerveModule(MechanismRoot2d visualRoot, String key, double radOffset, SwervePorts swervePorts, GenericSwerveModuleConstants individualConstants) {
        super(visualRoot, key, radOffset);
        super.pid = this.pid;
        super.motorConstants = this.motorConstants;
        super.moduleConstants = this.moduleConstants;
        super.moduleConstants.setEncoderInv(individualConstants.isEncoderInv());
        super.moduleConstants.setDrivingInv(individualConstants.isDrivingInv());
        super.moduleConstants.setTurningInv(individualConstants.isTurningInv());
        drive = MotorFactory.NEO(swervePorts.getDrivePort()).invert(super.moduleConstants.isDrivingInv());
        turn = MotorFactory.NEO550(swervePorts.getTurnPort()).invert(super.moduleConstants.isTurningInv());;
        absoluteEncoder = new AnalogInput5010(swervePorts.getEncoderPort());
        absoluteEncoder.setInverted(super.moduleConstants.isEncoderInv());
        setupSwerveEncoders();
    }    

}
