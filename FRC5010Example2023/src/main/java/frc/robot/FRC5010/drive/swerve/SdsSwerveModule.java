// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.swerve;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
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
    // private final MechanismLigament2d motorDial;
    // private final MechanismLigament2d absEncDial;
    // private final MechanismLigament2d expectDial;

    public SdsSwerveModule(MechanismRoot2d visualRoot, String key, double radOffset, SwerveConstants swerveConstants,
            SwervePorts swervePorts, Mk4iSwerveModuleHelper.GearRatio gearing) {
        super(visualRoot, key, radOffset, swerveConstants);
        constants = swerveConstants;
        ShuffleboardTab tab = Shuffleboard.getTab("SDS Drive");
        module = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout(getKey(), BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                gearing,
                swervePorts.getDrivePort(), swervePorts.getTurnPort(), swervePorts.getEncoderPort(), radOffset);

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

    public double getAbsoluteEncoderRad() {
        return module.getSteerAngle();
    }

    public void periodic() {
        // double turningDeg = Units.radiansToDegrees(getTurningPosition());
        // double absEncDeg = Units.radiansToDegrees(getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("Motor Ang: " + getKey(), turningDeg);
        // SmartDashboard.putNumber("Abs Angle: " + getKey(), absEncDeg);
        // SmartDashboard.putNumber("Abs Rads: " + getKey(), getAbsoluteEncoderRad());
        // // This method will be called once per scheduler run
        // absEncDial.setAngle(absEncDeg + 90);
        // motorDial.setAngle(turningDeg + 90);
        // motorDial.setLength(20 * getTurningVelocity() + 5);
        // expectDial.setLength(20 * getDriveVelocity() + 5);
        // expectDial.setAngle(getState().angle.getDegrees() + 90);
    }

}
