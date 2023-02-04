// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.constants.GenericMotorConstants;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.constants.GenericSwerveConstants;
import frc.robot.FRC5010.constants.GenericSwerveModuleConstants;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.sensors.AnalogInput5010;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;

/** Add your docs here. */
public abstract class GenericSwerveModule extends SubsystemBase {
    private final MechanismLigament2d motorDial;
    private final MechanismLigament2d absEncDial;
    private final MechanismLigament2d expectDial;
    private final String moduleKey;
    private ProfiledPIDController turningController;
    
    protected MotorController5010 drive, turn;
    protected GenericEncoder turnEncoder, driveEncoder, absoluteEncoder;
    protected GenericPID pid = new GenericPID(0, 0, 0); 
    protected GenericMotorConstants motorConstants = new GenericMotorConstants(0, 0, 0);
    protected GenericSwerveModuleConstants moduleConstants = new GenericSwerveModuleConstants(Units.inchesToMeters(0), 0, false, 0, false, false); 
    private double radOffset;
    private GenericSwerveConstants swerveConstants;
    // protected GenericEncoder driveEncoder = new SimulatedEncoder(0, 1); 
    // protected GenericEncoder turnEncoder = new SimulatedEncoder(2, 3);
    // protected GenericGyro gyro = new SimulatedGyro();

    public GenericSwerveModule(MechanismRoot2d visualRoot, String key, double radOffset, GenericSwerveConstants swerveConstants) {
        this.moduleKey = key;
        visualRoot.append(
            new MechanismLigament2d(moduleKey + "vert", 10, 90, 6.0, new Color8Bit(50, 50, 50)));
        visualRoot.append(
            new MechanismLigament2d(moduleKey + "hori", 10, 0, 6.0, new Color8Bit(50, 50, 50)));
        motorDial = visualRoot.append(
                new MechanismLigament2d(moduleKey, 10.0, 90, 6.0, new Color8Bit(Color.kYellow)));
        absEncDial = visualRoot.append(
                new MechanismLigament2d(moduleKey + "Abs", 10, 90, 6, new Color8Bit(Color.kBlue)));
        expectDial = visualRoot.append(
                new MechanismLigament2d(moduleKey + "Exp", 10, 90, 6, new Color8Bit(Color.kRed)));     
                
        this.radOffset = radOffset;
        this.swerveConstants = swerveConstants;

        
    }

    public void setupSwerveEncoders() {
        turnEncoder = turn.getMotorEncoder();
        driveEncoder = drive.getMotorEncoder();

        // set units drive encoder to meters and meters/sec
        driveEncoder.setPositionConversion(moduleConstants.getkDriveEncoderRot2Meter());
        driveEncoder.setVelocityConversion(moduleConstants.getkDriveEncoderRPM2MeterPerSec());
        // set units turning encoder to radians and radians/sec
        turnEncoder.setPositionConversion(moduleConstants.getkTurningEncoderRot2Rad());
        turnEncoder.setVelocityConversion(moduleConstants.getkTurningEncoderRPM2RadPerSec());

        turningController = new ProfiledPIDController(
            pid.getkP(), 
            pid.getkI(), 
            pid.getkD(),
            new TrapezoidProfile.Constraints(swerveConstants.getkTeleDriveMaxAngularSpeedRadiansPerSecond(), swerveConstants.getkTeleDriveMaxAngularAccelerationUnitsPerSecond())
        );

        turningController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    public double getAbsoluteEncoderRad() {
        return absoluteEncoder.getPosition() - radOffset;
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public boolean setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        double turnPow = turningController.calculate(getTurningPosition(),state.angle.getRadians());


        if(Math.abs(state.speedMetersPerSecond) < 0.001 && Math.abs(turnPow) < .01){
            stop();
            return false;
        }
      
        drive.set(state.speedMetersPerSecond / swerveConstants.getkPhysicalMaxSpeedMetersPerSecond());
        
        turn.set(turnPow + (Math.signum(turnPow) * motorConstants.getkS()));
        SmartDashboard.putString("Swerve [" + getKey()  + "] state", 
        " Angle: " + state.angle.getDegrees() + 
        " Speed m/s: " + state.speedMetersPerSecond);
        return true;
    }

    public SwerveModuleState getState() {
        // TODO Auto-generated method stub
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void stop(){
        drive.set(0);
        turn.set(0);
      }

    public String getKey(){
        return moduleKey; 
    }


    public void periodic() {
        double turningDeg = Units.radiansToDegrees(getTurningPosition()) + 90;
        double absEncDeg = Units.radiansToDegrees(getAbsoluteEncoderRad()) + 90;
        SmartDashboard.putNumber("Motor Ang: " + moduleKey, turningDeg);
        SmartDashboard.putNumber("Abs Angle: " + moduleKey, absEncDeg);
        SmartDashboard.putNumber("Abs Rads: " + moduleKey, getAbsoluteEncoderRad());
        // This method will be called once per scheduler run
        absEncDial.setAngle(absEncDeg);
        motorDial.setAngle(turningDeg);
        motorDial.setLength(20 * getTurningVelocity() + 5);
        expectDial.setLength(20 * getDriveVelocity() + 5);
        expectDial.setAngle(getState().angle.getDegrees() + 90);
    }

    
}
