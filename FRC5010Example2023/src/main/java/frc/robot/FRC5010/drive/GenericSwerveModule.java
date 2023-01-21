// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class GenericSwerveModule extends SubsystemBase {
    private final MechanismLigament2d motorDial;
    private final MechanismLigament2d absEncDial;
    private final MechanismLigament2d expectDial;
    private final String moduleKey;
    // protected GenericEncoder driveEncoder = new SimulatedEncoder(0, 1); 
    // protected GenericEncoder turnEncoder = new SimulatedEncoder(2, 3);
    // protected GenericGyro gyro = new SimulatedGyro();

    public GenericSwerveModule(MechanismRoot2d visualRoot, String key) {
        this.moduleKey = key;
        motorDial = visualRoot.append(
                new MechanismLigament2d(moduleKey, 10.0, 90, 6.0, new Color8Bit(Color.kYellow)));
        absEncDial = visualRoot.append(
                new MechanismLigament2d(moduleKey + "Abs", 10, 90, 6, new Color8Bit(Color.kBlue)));
        expectDial = visualRoot.append(
                new MechanismLigament2d(moduleKey + "Exp", 10, 90, 6, new Color8Bit(Color.kRed)));     
    }

    public abstract void resetEncoders();

    public abstract SwerveModuleState getState();

    public abstract double getDrivePosition();

    public abstract double getTurningPosition();

    public abstract double getAbsoluteEncoderRad();

    public abstract double getTurningVelocity();

    public abstract double getDriveVelocity();

    public abstract boolean setState(SwerveModuleState state);
    
    public abstract void stop();

    public String getKey(){
        return moduleKey; 
    }

    @Override
    public void periodic() {
        double turningDeg = Units.radiansToDegrees(getTurningPosition()) + 90;
        double absEncDeg = Units.radiansToDegrees(getAbsoluteEncoderRad()) + 90;
        SmartDashboard.putNumber("Motor Ang: " + moduleKey, turningDeg);
        SmartDashboard.putNumber("Abs Angle: " + moduleKey, absEncDeg);
        SmartDashboard.putNumber("Abs Rads: " + moduleKey, getAbsoluteEncoderRad());
        // This method will be called once per scheduler run
        absEncDial.setAngle(absEncDeg);
        motorDial.setAngle(turningDeg);
        motorDial.setLength(20 * getTurningVelocity());
        expectDial.setLength(20 * getDriveVelocity());
        expectDial.setAngle(getState().angle.getDegrees() + 90);
    }

    
}
