// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.swerve;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;

/** Add your docs here. */
public class SdsSwerveModule extends GenericSwerveModule {
    private SwerveModule module;
    private final String moduleKey;
    private SwerveConstants constants;

    public SdsSwerveModule(MechanismRoot2d visualRoot, String key, double radOffset, SwerveConstants swerveConstants,
        SwervePorts swervePorts, Mk4iSwerveModuleHelper.GearRatio gearing) {
        super(visualRoot, key, radOffset, swerveConstants);
        moduleKey = key;
        constants = swerveConstants;
        ShuffleboardTab tab = Shuffleboard.getTab("SDS Drive");
        module = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout(moduleKey, BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                gearing,
                swervePorts.getDrivePort(), swervePorts.getTurnPort(), swervePorts.getEncoderPort(), radOffset);
    }

    public boolean setState(SwerveModuleState state, boolean ready) {
        module.set(
                state.speedMetersPerSecond / constants.getkPhysicalMaxSpeedMetersPerSecond()
                        * SdsSwerveDrivetrain.MAX_VOLTAGE,
                state.angle.getRadians());
        return true;
    }

    public double getTurningPosition() {
        return module.getSteerAngle();
    }

    public double getDriveVelocity() {
        return module.getDriveVelocity();
    }

    public double getDrivePosition() {
        return module.getDriveDistance();
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void periodic() {}

}
