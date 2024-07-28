// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

/** Add your docs here. */
public class GenericDrivetrainConstants {
	protected double DRIVETRAIN_TRACKWIDTH_METERS;
	protected double DRIVETRAIN_WHEELBASE_METERS;
	protected double kWheelDiameterMeters; // = Units.inchesToMeters(3);
	protected double kDriveMotorGearRatio;

	protected double kPhysicalMaxAngularSpeedRadiansPerSecond;
	protected double kPhysicalMaxSpeedMetersPerSecond;

	protected double kTeleDriveMaxSpeedMetersPerSecond;
	protected double kTeleDriveMaxAngularSpeedRadiansPerSecond;
	protected double kTeleDriveMaxAccelerationUnitsPerSecond;
	protected double kTeleDriveMaxAngularAccelerationUnitsPerSecond;

	public GenericDrivetrainConstants() {
		this.DRIVETRAIN_TRACKWIDTH_METERS = 0.0;
		this.DRIVETRAIN_WHEELBASE_METERS = 0.0;
		this.kWheelDiameterMeters = 0.0;
		this.kDriveMotorGearRatio = 0.0;
		this.kPhysicalMaxAngularSpeedRadiansPerSecond = 0.0;
		this.kPhysicalMaxSpeedMetersPerSecond = 0.0;
		this.kTeleDriveMaxSpeedMetersPerSecond = 0.0;
		this.kTeleDriveMaxAngularSpeedRadiansPerSecond = 0.0;
		this.kTeleDriveMaxAccelerationUnitsPerSecond = 0.0;
		this.kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0.0;
	}	
	
	public GenericDrivetrainConstants(GenericDrivetrainConstants constants) {
		this.DRIVETRAIN_TRACKWIDTH_METERS = constants.DRIVETRAIN_TRACKWIDTH_METERS;
		this.DRIVETRAIN_WHEELBASE_METERS = constants.DRIVETRAIN_WHEELBASE_METERS;
		this.kWheelDiameterMeters = constants.kWheelDiameterMeters;
		this.kDriveMotorGearRatio = constants.kDriveMotorGearRatio;
		this.kPhysicalMaxAngularSpeedRadiansPerSecond = constants.kPhysicalMaxAngularSpeedRadiansPerSecond;
		this.kPhysicalMaxSpeedMetersPerSecond = constants.kPhysicalMaxSpeedMetersPerSecond;
		this.kTeleDriveMaxSpeedMetersPerSecond = constants.kTeleDriveMaxSpeedMetersPerSecond;
		this.kTeleDriveMaxAngularSpeedRadiansPerSecond = constants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
		this.kTeleDriveMaxAccelerationUnitsPerSecond = constants.kTeleDriveMaxAccelerationUnitsPerSecond;
		this.kTeleDriveMaxAngularAccelerationUnitsPerSecond = constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;
	}

	public void setTrackWidth(double trackWidth) {
		this.DRIVETRAIN_TRACKWIDTH_METERS = trackWidth;
	}

	public double getTrackWidth() {
		return DRIVETRAIN_TRACKWIDTH_METERS;
	}

	public void setWheelBase(double wheelBase) {
		this.DRIVETRAIN_WHEELBASE_METERS = wheelBase;
	}

	public double getWheelBase() {
		return DRIVETRAIN_WHEELBASE_METERS;
	}

	public void setWheelDiameter(double wheelDiameter) {
		this.kWheelDiameterMeters = wheelDiameter;
	}

	public double getWheelDiameter() {
		return kWheelDiameterMeters;
	}

	public double getkPhysicalMaxAngularSpeedRadiansPerSecond() {
		return kPhysicalMaxAngularSpeedRadiansPerSecond;
	}

	public void setkPhysicalMaxAngularSpeedRadiansPerSecond(
			double kPhysicalMaxAngularSpeedRadiansPerSecond) {
		this.kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
	}

	public double getkPhysicalMaxSpeedMetersPerSecond() {
		return kPhysicalMaxSpeedMetersPerSecond;
	}

	public void setkPhysicalMaxSpeedMetersPerSecond(double kPhysicalMaxSpeedMetersPerSecond) {
		this.kPhysicalMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
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

	public void setkTeleDriveMaxAngularSpeedRadiansPerSecond(
			double kTeleDriveMaxAngularSpeedRadiansPerSecond) {
		this.kTeleDriveMaxAngularSpeedRadiansPerSecond = kTeleDriveMaxAngularSpeedRadiansPerSecond;
	}

	public double getkTeleDriveMaxAccelerationUnitsPerSecond() {
		return kTeleDriveMaxAccelerationUnitsPerSecond;
	}

	public void setkTeleDriveMaxAccelerationUnitsPerSecond(
			double kTeleDriveMaxAccelerationUnitsPerSecond) {
		this.kTeleDriveMaxAccelerationUnitsPerSecond = kTeleDriveMaxAccelerationUnitsPerSecond;
	}

	public double getkTeleDriveMaxAngularAccelerationUnitsPerSecond() {
		return kTeleDriveMaxAngularAccelerationUnitsPerSecond;
	}

	public void setkTeleDriveMaxAngularAccelerationUnitsPerSecond(
			double kTeleDriveMaxAngularAccelerationUnitsPerSecond) {
		this.kTeleDriveMaxAngularAccelerationUnitsPerSecond = kTeleDriveMaxAngularAccelerationUnitsPerSecond;
	}

	public double getkDriveMotorGearRatio() {
		return kDriveMotorGearRatio;
	}

	public void setkDriveMotorGearRatio(double kDriveMotorGearRatio) {
		this.kDriveMotorGearRatio = kDriveMotorGearRatio;
	}
}
