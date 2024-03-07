// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FRC5010.arch.GenericSubsystem;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;

/** Add your docs here. */
public class TargetingSystem extends GenericSubsystem {
    private Supplier<Pose3d> currentTarget;
    // Velocity Extrapolation
    private Pose3d extrapolatedTarget = new Pose3d();
    private int convergence_trials = 10;


    

    private Supplier<Pose3d> robotPose;
    private SwerveDrivetrain swerve;

    private PIDController thetaController;
    // Input/Output Values
    private final String kP = "Targeting kP";
    private final String kI = "Targeting kI";
    private final String kD = "Targeting kD";
    private final String TOLERANCE = "Targeting Tolerance";

    private static InterpolatingDoubleTreeMap pivotInterpolation = new InterpolatingDoubleTreeMap();

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
        values.declare(kI, 0.0);
        values.declare(kD, 0.005);
        values.declare(TOLERANCE, 0.03);

        values.declare(TURN_POWER, 0.0);
        values.declare(HORIZONTAL_ANGLE, 0.0);
        values.declare(PIVOT_ANGLE, 0.0);
    

        // Initialize the PID controller
        thetaController = new PIDController(values.getDouble(kP), values.getDouble(kI), values.getDouble(kD));
        thetaController.setTolerance(values.getDouble(TOLERANCE));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        pivotInterpolation.put(0.95, -10.0);
        pivotInterpolation.put(1.52, 0.67);
        pivotInterpolation.put(2.004, 7.00);
        pivotInterpolation.put(2.57, 12.65);
        pivotInterpolation.put(3.07, 17.1);
        pivotInterpolation.put(3.506, 19.66);
        pivotInterpolation.put(4.07, 21.422);
    }

    public static Pose3d getSpeakerTarget(Alliance alliance) {
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

    public double getLaunchVelocity(double shooterVelocity) {
        return Units.feetToMeters(38);
    }

    public void updateControllerValues() {
        thetaController.setP(values.getDouble(kP));
        thetaController.setI(values.getDouble(kI));
        thetaController.setD(values.getDouble(kD));
        thetaController.setTolerance(values.getDouble(TOLERANCE));
    }

    public boolean isAtTargetYaw() {
        return thetaController.atSetpoint();
    }

    public double getPivotAngle() {
        double angle = interpolatePivotAngle(currentTarget.get(), robotPose.get());
        values.set(PIVOT_ANGLE, angle);
        return angle;
    }

    public double getPivotAngle(double launchVelocity) {
        // TODO: Make sure extrapolated target is updated, and don't call it twice to waste cpu
        double angle = interpolatePivotAngle(extrapolatedTarget, robotPose.get());
        values.set(PIVOT_ANGLE, angle);
        return angle;
    }

    /* 
    Predicts and sets predicted target position based on time for note to reach target and robot velocity
    */ 
    public void extrapolateTargetPosition(double launchVelocity, ChassisSpeeds robotVelocity, Pose3d robotPosition) {
        double speaker_x_velocity = -robotVelocity.vxMetersPerSecond;
        double speaker_y_velocity = -robotVelocity.vyMetersPerSecond;
        Translation3d predicted_target_location = currentTarget.get().getTranslation();
        
        for (int i = 0; i<convergence_trials; i++) {
            // Gets current distance to target
            double distance = predicted_target_location.getDistance(robotPose.get().getTranslation());
            // Pivot Angle
            // double pivotAngle = interpolatePivotAngle(predicted_target_location.toTranslation2d().getDistance(robotPose.get().getTranslation().toTranslation2d()));
            // TODO: Add constant to get pivot angle from ground and then calculate horizontal velocity
            // Get time to target
            double time = distance / launchVelocity;
            // apply time to target to speaker position
            predicted_target_location = new Translation3d(currentTarget.get().getX() + speaker_x_velocity * time, currentTarget.get().getY() + speaker_y_velocity * time, currentTarget.get().getZ());
            
        }
        extrapolatedTarget = new Pose3d(predicted_target_location, currentTarget.get().getRotation());
    } 

    

    public double getHorizontalAngle() {
        double x = currentTarget.get().getTranslation().getX() - robotPose.get().getTranslation().getX();
        double y = currentTarget.get().getTranslation().getY() - robotPose.get().getTranslation().getY();
        double static_angle = Math.atan2(y, x);
        values.set(HORIZONTAL_ANGLE, Units.radiansToDegrees(static_angle));
        return Units.radiansToDegrees(static_angle);

    }

    public double getHorizontalAngle(double launchVelocity) {
        ChassisSpeeds speeds = swerve.getChassisSpeeds();
        // Extrapolate target position based on robot velocity
        extrapolateTargetPosition(launchVelocity, speeds, robotPose.get());
        double angle = Math.atan2(extrapolatedTarget.getY() - robotPose.get().getTranslation().getY(), extrapolatedTarget.getX() - robotPose.get().getTranslation().getX());

        values.set(HORIZONTAL_ANGLE, angle);
        return Units.radiansToDegrees(angle);

    }

    public double getPivotAngleToTarget() { // TODO: Account for gravity
        double x = currentTarget.get().getTranslation().getX() - robotPose.get().getTranslation().getX();
        double y = currentTarget.get().getTranslation().getY() - robotPose.get().getTranslation().getY();
        double angle = Math.atan2(y, x);
        values.set(PIVOT_ANGLE, angle);
        return Units.radiansToDegrees(angle);
    }

    public static double interpolatePivotAngle(double distance) {
        return pivotInterpolation.get(distance);
    }

    public static double interpolatePivotAngle(Pose3d target, Pose3d robot) {
        return pivotInterpolation.get(target.getTranslation().toTranslation2d().getDistance(robot.getTranslation().toTranslation2d()));
    }

    public double getTurnPower() {
        double targetAngle = getHorizontalAngle(getLaunchVelocity(1.0));
        updateControllerValues();
        thetaController.setSetpoint(Units.degreesToRadians(targetAngle));
        return thetaController.atSetpoint() ? 0.0 : thetaController.calculate(robotPose.get().getRotation().getZ())
                * swerve.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond(); // Stops rotating robot  once at setpoint within tolerance.
    }

}
