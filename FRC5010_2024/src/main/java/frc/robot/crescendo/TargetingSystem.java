// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FRC5010.arch.GenericSubsystem;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;

/** Add your docs here. */
public class TargetingSystem extends GenericSubsystem {
    private Supplier<Pose3d> currentTarget;
    private Supplier<Pose3d> robotPose;
    private SwerveDrivetrain swerve;

    private PIDController thetaController;
    // Input/Output Values
    private final String kP = "Targeting kP";
    private final String kI = "Targeting kI";
    private final String kD = "Targeting kD";
    private final String TOLERANCE = "Targeting Tolerance";

    // Output Values
    private final String TURN_POWER = "Turn Power";
    private final String HORIZONTAL_ANGLE = "Horizontal Angle";
    private final String PIVOT_ANGLE = "Pivot Angle";


    public TargetingSystem(Supplier<Pose3d> targetSupplier, Supplier<Pose3d> robotPose, SwerveDrivetrain swerve) {
        this.currentTarget = targetSupplier;
        this.robotPose = robotPose;
        this.swerve = swerve;

        // Declare Values
        values.declare(kP, 0.25);
        values.declare(kI, 0);
        values.declare(kD, 0.005);
        values.declare(TOLERANCE, 0.05);

        values.declare(TURN_POWER, 0.0);
        values.declare(HORIZONTAL_ANGLE, 0.0);
        values.declare(PIVOT_ANGLE, 0.0);
    

        // Initialize the PID controller
        thetaController = new PIDController(values.getDouble(kP), values.getDouble(kI), values.getDouble(kD));
        thetaController.setTolerance(values.getDouble(TOLERANCE));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Pose3d getSpeakerTarget(Alliance alliance) {
        return alliance == Alliance.Blue ? Constants.Field.BLUE_SHOT_POSE : Constants.Field.RED_SHOT_POSE;
    }

    public void setTarget(Pose3d target) {
        currentTarget = () -> target;
    }

    public Pose3d getTarget() {
        return currentTarget.get();
    }

    public void init() {
        thetaController.reset();
    }

    public void updateControllerValues() {
        thetaController.setP(values.getDouble(kP));
        thetaController.setI(values.getDouble(kI));
        thetaController.setD(values.getDouble(kD));
        thetaController.setTolerance(values.getDouble(TOLERANCE));
    }

    public double getPivotAngle() {
        return 0.0;
    }

    private static double vectorAngleDifference(double x1, double y1, double x2, double y2) {
        return Math.atan2(x1 * y2 - y1 * x2, dotProduct2d(x1, y1, x2, y2));
    }
    
    private static double dotProduct2d(double x1, double y1, double x2, double y2) {
        return x1 * x2 + y1 * y2;
    }


    public double getHorizontalAngle(boolean AccountForVelocity, double launchVelocity) {
        ChassisSpeeds speeds = AccountForVelocity ? swerve.getChassisSpeeds() : new ChassisSpeeds();
        double x = currentTarget.get().getTranslation().getX() - robotPose.get().getTranslation().getX();
        double y = currentTarget.get().getTranslation().getY() - robotPose.get().getTranslation().getY();
        double static_angle = Math.atan(y / x);
        double note_x = launchVelocity * Math.cos(static_angle);
        double note_y = launchVelocity * Math.sin(static_angle);
        double x_total = note_x + speeds.vxMetersPerSecond;
        double y_total = note_y + speeds.vyMetersPerSecond;
        double moving_angle = vectorAngleDifference(x, y, x_total, y_total);
        double accounted_angle = static_angle - moving_angle;
        values.set(HORIZONTAL_ANGLE, accounted_angle);
        return Units.radiansToDegrees(accounted_angle);

    }

    public double getPivotAngleToTarget() { // TODO: Account for gravity
        double x = currentTarget.get().getTranslation().getX() - robotPose.get().getTranslation().getX();
        double y = currentTarget.get().getTranslation().getY() - robotPose.get().getTranslation().getY();
        double angle = Math.atan2(y, x);
        values.set(PIVOT_ANGLE, angle);
        return Units.radiansToDegrees(angle);
    }

    public double getTurnPower() {
        double targetAngle = 0.0;
        updateControllerValues();
        thetaController.setSetpoint(targetAngle);
        return thetaController.atSetpoint() ? 0.0 : thetaController.calculate(robotPose.get().getRotation().getZ())
                * swerve.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond(); // Stops rotating robot  once at setpoint within tolerance.
    }

}
