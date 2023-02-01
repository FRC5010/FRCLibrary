// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class GenericSwerveConstants extends GenericDrivetrainConstants{
    // public variables
    private double DRIVETRAIN_TRACKWIDTH_METERS;
    private double DRIVETRAIN_WHEELBASE_METERS;

    private SwerveDriveKinematics kinematics;

    

    private double kFrontLeftAbsoluteOffsetRad; 
    private double kFrontRightAbsoluteOffsetRad; 
    private double kBackLeftAbsoluteOffsetRad; 
    private double kBackRightAbsoluteOffsetRad;

    private double kPhysicalMaxSpeedMetersPerSecond;
    private double kPhysicalMaxAngularSpeedRadiansPerSecond;

    private double kTeleDriveMaxSpeedMetersPerSecond;
    private double kTeleDriveMaxAngularSpeedRadiansPerSecond;
    private double kTeleDriveMaxAccelerationUnitsPerSecond;
    private double kTeleDriveMaxAngularAccelerationUnitsPerSecond;

    public GenericSwerveConstants(double DRIVETRAIN_TRACKWIDTH_METERS, double DRIVETRAIN_WHEELBASE_METERS){
        this.DRIVETRAIN_TRACKWIDTH_METERS = DRIVETRAIN_TRACKWIDTH_METERS;
        this.DRIVETRAIN_WHEELBASE_METERS = DRIVETRAIN_WHEELBASE_METERS;

    
        kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getkFrontLeftAbsoluteOffsetRad() {
        return kFrontLeftAbsoluteOffsetRad;
    }

    public void setkFrontLeftAbsoluteOffsetRad(double kFrontLeftAbsoluteOffsetRad) {
        this.kFrontLeftAbsoluteOffsetRad = kFrontLeftAbsoluteOffsetRad;
    }

    public double getkFrontRightAbsoluteOffsetRad() {
        return kFrontRightAbsoluteOffsetRad;
    }

    public void setkFrontRightAbsoluteOffsetRad(double kFrontRightAbsoluteOffsetRad) {
        this.kFrontRightAbsoluteOffsetRad = kFrontRightAbsoluteOffsetRad;
    }

    public double getkBackLeftAbsoluteOffsetRad() {
        return kBackLeftAbsoluteOffsetRad;
    }

    public void setkBackLeftAbsoluteOffsetRad(double kBackLeftAbsoluteOffsetRad) {
        this.kBackLeftAbsoluteOffsetRad = kBackLeftAbsoluteOffsetRad;
    }

    public double getkBackRightAbsoluteOffsetRad() {
        return kBackRightAbsoluteOffsetRad;
    }

    public void setkBackRightAbsoluteOffsetRad(double kBackRightAbsoluteOffsetRad) {
        this.kBackRightAbsoluteOffsetRad = kBackRightAbsoluteOffsetRad;
    }

    public double getkPhysicalMaxSpeedMetersPerSecond() {
        return kPhysicalMaxSpeedMetersPerSecond;
    }

    public void setkPhysicalMaxSpeedMetersPerSecond(double kPhysicalMaxSpeedMetersPerSecond) {
        this.kPhysicalMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    }

    public double getkPhysicalMaxAngularSpeedRadiansPerSecond() {
        return kPhysicalMaxAngularSpeedRadiansPerSecond;
    }

    public void setkPhysicalMaxAngularSpeedRadiansPerSecond(double kPhysicalMaxAngularSpeedRadiansPerSecond) {
        this.kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    }

    public double getkTeleDriveMaxSpeedMetersPerSecond() {
        return kTeleDriveMaxSpeedMetersPerSecond;
    }

    public void setkTeleDriveMaxSpeedMetersPerSecond(double kTeleDriveMaxSpeedMetersPerSecond) {
        this.kTeleDriveMaxSpeedMetersPerSecond = kTeleDriveMaxSpeedMetersPerSecond;
    }

    public double getkTeleDriveMaxAngularSpeedRadiansPerSecond() {
        return kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }

    public void setkTeleDriveMaxAngularSpeedRadiansPerSecond(double kTeleDriveMaxAngularSpeedRadiansPerSecond) {
        this.kTeleDriveMaxAngularSpeedRadiansPerSecond = kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }

    public double getkTeleDriveMaxAccelerationUnitsPerSecond() {
        return kTeleDriveMaxAccelerationUnitsPerSecond;
    }

    public void setkTeleDriveMaxAccelerationUnitsPerSecond(double kTeleDriveMaxAccelerationUnitsPerSecond) {
        this.kTeleDriveMaxAccelerationUnitsPerSecond = kTeleDriveMaxAccelerationUnitsPerSecond;
    }

    public double getkTeleDriveMaxAngularAccelerationUnitsPerSecond() {
        return kTeleDriveMaxAngularAccelerationUnitsPerSecond;
    }

    public void setkTeleDriveMaxAngularAccelerationUnitsPerSecond(double kTeleDriveMaxAngularAccelerationUnitsPerSecond) {
        this.kTeleDriveMaxAngularAccelerationUnitsPerSecond = kTeleDriveMaxAngularAccelerationUnitsPerSecond;
    }

    

}
