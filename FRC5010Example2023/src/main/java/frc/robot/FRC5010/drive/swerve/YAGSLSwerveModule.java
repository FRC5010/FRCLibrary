// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.swerve;

import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveModule;

/** Add your docs here. */
public class YAGSLSwerveModule extends GenericSwerveModule{
    private SwerveModule swerveModule; 

    public YAGSLSwerveModule(SwerveModule swerveModule, MechanismRoot2d visual, String key){
        super(visual, key, 0, null); 
        this.swerveModule = swerveModule; 
    } 

    public double getDrivePosition() {
        return swerveModule.getPosition().distanceMeters;
    }

    public double getTurningPosition() {
        return swerveModule.getPosition().angle.getRadians();
    }

    public double getAbsoluteEncoderRad() {
        return swerveModule.getAbsolutePosition();
    }

}
