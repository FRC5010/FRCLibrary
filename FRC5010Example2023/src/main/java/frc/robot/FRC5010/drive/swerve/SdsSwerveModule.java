// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.swerve;

import java.util.Timer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.swervelib.AbsoluteEncoder;
import frc.robot.swervelib.MechanicalConfiguration;
import frc.robot.swervelib.MkSwerveModuleBuilder;
import frc.robot.swervelib.MotorType;
import frc.robot.swervelib.SwerveModule;

/** Add your docs here. */
public class SdsSwerveModule extends GenericSwerveModule {
    private SwerveModule module;
    private SwerveConstants constants;
    private AbsoluteEncoder absEncoder;
    public static final double MAX_VOLTAGE = 12.0;

    // private final MechanismLigament2d motorDial;
    // private final MechanismLigament2d absEncDial;
    // private final MechanismLigament2d expectDial;

    public SdsSwerveModule(MechanismRoot2d visualRoot, String key, double radOffset, SwerveConstants swerveConstants,
            SwervePorts swervePorts, MechanicalConfiguration gearing) {
        super(visualRoot, key, radOffset, swerveConstants);
        constants = swerveConstants;
        ShuffleboardTab tab = Shuffleboard.getTab("SDS Drive");
        module = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout(getKey(), BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0))
                .withGearRatio(gearing)
                .withDriveMotor(MotorType.NEO, swervePorts.getDrivePort())
                .withSteerMotor(MotorType.NEO, swervePorts.getTurnPort())
                .withSteerEncoderPort(swervePorts.getEncoderPort())
                .withSteerOffset(radOffset)
                .build();
        try{ 
            new Timer().wait(1000);
            System.out.println("Reset");
        } catch (Exception e){
            
        } 
        module.resetToAbsolute();
        // new ScheduleCommand(Commands.waitSeconds(1.0)).andThen(Commands.runOnce(() -> { System.out.println("reset");})); 

        System.out.println("Printing");
        absEncoder = module.getSteerEncoder();
    }

    @Override
    public boolean setState(SwerveModuleState state, boolean ready) {
        module.set(
                state.speedMetersPerSecond / constants.getkPhysicalMaxSpeedMetersPerSecond()
                        * MAX_VOLTAGE,
                state.angle.getRadians());
        return true;
    }

    @Override
    public double getTurningPosition() {
        return module.getSteerAngle();
    }

    @Override
    public double getTurningVelocity() {
        return 0;
    }

    @Override
    public double getDriveVelocity() {
        return module.getDriveVelocity();
    }

    @Override
    public double getDrivePosition() {
        return module.getDriveDistance();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getDriveVelocity(), new Rotation2d(this.getTurningPosition()));
    }

    @Override
    public double getAbsoluteEncoderRad() {
        return absEncoder.getAbsoluteAngle();
    }

}
