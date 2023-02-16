// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.swerve;

import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;

/** Add your docs here. */
public class SdsSwerveModule extends GenericSwerveModule {
    private SwerveModule module;
    private SwerveConstants constants;
    private AbsoluteEncoder absEncoder;
    private double radOffset;

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
            .build()
        ;

        this.radOffset = radOffset;
        absEncoder = module.getSteerEncoder();


        // module = Mk4iSwerveModuleHelper.createNeo(
        //         tab.getLayout(getKey(), BuiltInLayouts.kList)
        //                 .withSize(2, 4)
        //                 .withPosition(0, 0),
        //         gearing,
        //         swervePorts.getDrivePort(), swervePorts.getTurnPort(), swervePorts.getEncoderPort(), radOffset);

        // visualRoot.append(
        //         new MechanismLigament2d(getKey() + "vert", 10, 90, 6.0, new Color8Bit(50, 50, 50)));
        // visualRoot.append(
        //         new MechanismLigament2d(getKey() + "hori", 10, 0, 6.0, new Color8Bit(50, 50, 50)));
        // motorDial = visualRoot.append(
        //         new MechanismLigament2d(getKey(), 10.0, 90, 6.0, new Color8Bit(Color.kYellow)));
        // absEncDial = visualRoot.append(
        //         new MechanismLigament2d(getKey() + "Abs", 10, 90, 6, new Color8Bit(Color.kBlue)));
        // expectDial = visualRoot.append(
        //         new MechanismLigament2d(getKey() + "Exp", 10, 90, 6, new Color8Bit(Color.kRed)));
    }

    @Override
    public boolean setState(SwerveModuleState state, boolean ready) {        
        module.set(
                state.speedMetersPerSecond / constants.getkPhysicalMaxSpeedMetersPerSecond()
                        * SdsSwerveDrivetrain.MAX_VOLTAGE,
                state.angle.getRadians());
        return true;
    }
    @Override
    public double getTurningPosition() {
        return module.getSteerAngle();
    }
    @Override
    public double getTurningVelocity(){
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
