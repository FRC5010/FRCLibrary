// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.mechanisms.DriveConstantsDef;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ****** Actual persisted values - use camelCaseNamingStyle ************
    public Persisted<Double> maxChassisVelocity;
    public Persisted<Double> maxChassisRotation;

    // Create our feedforward gain constants (from the identification tool)
    public Persisted<Double> kvVoltSecondsPerMeter;
    public Persisted<Double> kaVoltSecondsSquaredPerMeter;
    public Persisted<Double> motorRotationsPerWheelRotation;
    public Persisted<Double> trackWidth;

    public Persisted<Double> kVAngular;
    public Persisted<Double> kAAngular;

    public Persisted<String> whoAmI;

    public Constants() {
        whoAmI = new Persisted<>(RobotContainer.WHO_AM_I, "");
        // ****** Actual persisted values - use camelCaseNamingStyle ************
        maxChassisVelocity = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_VELOCITY, 15.0);
        maxChassisRotation = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_ROTATION, 1.5);

        // Create our feedforward gain constants (from the identification tool)
        kvVoltSecondsPerMeter = new Persisted<>(DriveConstantsDef.KV_DRIVE_LINEAR, 1.3984);
        kaVoltSecondsSquaredPerMeter = new Persisted<>(DriveConstantsDef.KA_DRIVE_LINEAR, 0.27137);
        motorRotationsPerWheelRotation = new Persisted<>(DriveConstantsDef.MOTOR_ROT_PER_WHEEL_ROT, 10.71);
        trackWidth = new Persisted<>(DriveConstantsDef.TRACK_WIDTH, 0.616);

        kVAngular = new Persisted<>(DriveConstantsDef.KV_DRIVE_ANGULAR, 0.5 / 180);
        kAAngular = new Persisted<>(DriveConstantsDef.KA_DRIVE_ANGULAR, 0.03);
    }
}
