// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FRC5010.motors.MotorController5010;
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
    

    public ThriftySwerveModule(MechanismRoot2d visualRoot, String key, MotorController5010 drive, MotorController5010 turn, double radOffset, GenericEncoder analogEncoder) {
        super(visualRoot, key);
        this.drive = drive; 
        this.turn = turn;
        this.turnEncoder = turn.getMotorEncoder();
        this.driveEncoder = drive.getMotorEncoder();
        this.absoluteEncoder = analogEncoder;

        this.radOffset = radOffset;

        // set units drive encoder to meters and meters/sec
        driveEncoder.setPositionConversion(kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversion(kDriveEncoderRPM2MeterPerSec);
        // set units turning encoder to radians and radians/sec
        turnEncoder.setPositionConversion(kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversion(kTurningEncoderRPM2RadPerSec);
        
        turningController = new ProfiledPIDController(
            kPTurning, 
            kITurning, 
            kDTurning,
            new TrapezoidProfile.Constraints(SwerveDrivetrain.kTeleDriveMaxAngularSpeedRadiansPerSecond, SwerveDrivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond)
        );

        turningController.enableContinuousInput(-Math.PI, Math.PI);

        
        new Thread(() -> {
            try{
              Thread.sleep(1000);
            }catch(Exception e){}
            resetEncoders();
          }).start();
    }

    
    @Override
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    @Override
    public double getDrivePosition() {
        // TODO Auto-generated method stub
        return driveEncoder.getPosition();
    }

    @Override
    public double getTurningPosition() {
        // TODO Auto-generated method stub
        return turnEncoder.getPosition();
    }

    @Override
    public double getAbsoluteEncoderRad() {
        // TODO Auto-generated method stub
        return absoluteEncoder.getPosition() - radOffset;
    }

    @Override
    public double getTurningVelocity() {
        // TODO Auto-generated method stub
        return turnEncoder.getVelocity();
    }

    @Override
    public double getDriveVelocity() {
        // TODO Auto-generated method stub
        return driveEncoder.getVelocity();
    }

    @Override
    public boolean setState(SwerveModuleState state) {
        // TODO Auto-generated method stub
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return false;
          }
      
        state = SwerveModuleState.optimize(state, getState().angle);
        drive.set(state.speedMetersPerSecond / SwerveDrivetrain.kPhysicalMaxSpeedMetersPerSecond);
        //expectDial.setLength(50 * drive.get());
        double turnPow = turningController.calculate(getTurningPosition(),state.angle.getRadians());
        // getTurningPosition()
        // adding ks to get swerve moving
        turn.set(turnPow + (Math.signum(turnPow) * kS));
        SmartDashboard.putString("Swerve [" + super.getKey()  + "] state", 
        "Angle: " + state.angle.getDegrees() + " Speed m/s: " + state.speedMetersPerSecond);
        return true;
    }

    @Override
    public SwerveModuleState getState() {
        // TODO Auto-generated method stub
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    @Override
    public void stop(){
        drive.set(0);
        turn.set(0);
      }

      @Override
      public void periodic() {
          // TODO Auto-generated method stub
          super.periodic();
      }

    

}
