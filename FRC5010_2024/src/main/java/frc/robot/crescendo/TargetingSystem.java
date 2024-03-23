// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.arch.GenericSubsystem;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;

/** Add your docs here. */
public class TargetingSystem extends GenericSubsystem {
    private Supplier<Pose3d> currentTarget;
    // Velocity Extrapolation
    private Pose3d extrapolatedTarget = new Pose3d();
    private int convergence_trials = 10;
    private boolean ranExtrapolation = false;

    private Supplier<Pose3d> robotPose;
    private SwerveDrivetrain swerve;
    private VisionSystem shooterCamera;

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

    private final double DEFAULT_TOLERANCE = 0.01;

    public TargetingSystem(Supplier<Pose3d> targetSupplier, Supplier<Pose3d> robotPose, SwerveDrivetrain swerve,
            VisionSystem shooter) {
        this.currentTarget = targetSupplier;
        this.robotPose = robotPose;
        this.swerve = swerve;
        this.shooterCamera = shooter;

        // Declare Values
        values.declare(kP, 0.28);
        values.declare(kI, 0.0);
        values.declare(kD, 0.04);
        values.declare(TOLERANCE, DEFAULT_TOLERANCE);

        values.declare(TURN_POWER, 0.0);
        values.declare(HORIZONTAL_ANGLE, 0.0);
        values.declare(PIVOT_ANGLE, 0.0);

        // Initialize the PID controller
        thetaController = new PIDController(values.getDouble(kP), values.getDouble(kI), values.getDouble(kD));
        thetaController.setTolerance(values.getDouble(TOLERANCE));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        pivotInterpolation.put(0.92794, -11.0); // -9.15
        pivotInterpolation.put(1.0791, -5.5); // -4.55
        pivotInterpolation.put(1.80, -2.08);
        pivotInterpolation.put(2.08, 5.62); // 13.713
        pivotInterpolation.put(2.23, 8.91); // 13.713
        pivotInterpolation.put(2.48, 10.567); // 19.35
        pivotInterpolation.put(2.93, 19.03);
        pivotInterpolation.put(3.05, 19.38);
        pivotInterpolation.put(3.24, 21.556);
        pivotInterpolation.put(3.58, 21.28);
        pivotInterpolation.put(3.87, 23.67);
        pivotInterpolation.put(4.58, 26.40);
        pivotInterpolation.put(5.00, 28.86);
        
        // pivotInterpolation.put(3.25, 24.25); // 26.01
        // pivotInterpolation.put(3.54, 26.0); // 28.11
        // pivotInterpolation.put(3.75, 27.25); // -4.55
        // pivotInterpolation.put(4.0, 28.25); // -2.09
        // pivotInterpolation.put(4.25, 29.25); // 3.885
        // pivotInterpolation.put(4.5, 29.5); // 8.799
        // pivotInterpolation.put(4.75, 30.5); // 13.713
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

    public void setTolerance(double value) {
        values.set(TOLERANCE, value);
    }

    public void resetToleranceToDefaults() {
        values.set(TOLERANCE, DEFAULT_TOLERANCE);
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

    public double getStraightLinePivotAngle() {
        double distance = getFlatDistanceToTarget();
        double z = currentTarget.get().getTranslation().getZ();
        return Units.radiansToDegrees(Math.atan2(z, distance));
    }

    public double getPivotAngle() {
        double angle = interpolatePivotAngle(currentTarget.get(), robotPose.get());
        values.set(PIVOT_ANGLE, angle);
        return angle;
    }

    public double getPivotAngle(double launchVelocity) {
        ChassisSpeeds speeds = swerve.getChassisSpeeds();
        extrapolateTargetPosition(launchVelocity, speeds, extrapolatedTarget);
        double angle = interpolatePivotAngle(extrapolatedTarget, robotPose.get());
        values.set(PIVOT_ANGLE, angle);
        return angle;
    }

    public static double getShooterAngle(double pivotAngle) {
        return Constants.Physical.SHOOTER_ANGLE_OFFSET - pivotAngle;
    }

    public static double getAngleToExitPoint(double pivotAngle) {
        return Constants.Physical.PIVOT_EXIT_POINT_ANGLE_OFFSET - pivotAngle;
    }

    public static Translation3d getExitOffset(double pivotAngle) {
        return new Translation3d(Constants.Physical.PIVOT_SHOOTER_RADIUS, 0.0, 0.0)
                .rotateBy(new Rotation3d(Units.degreesToRadians(getAngleToExitPoint(pivotAngle)), 0, 0));
    }

    /*
     * Predicts and sets predicted target position based on time for note to reach
     * target and robot velocity
     */
    public void extrapolateTargetPosition(double launchVelocity, ChassisSpeeds robotVelocity, Pose3d robotPosition) {
        if (ranExtrapolation) {
            return;
        }
        double speaker_x_velocity = -robotVelocity.vxMetersPerSecond;
        double speaker_y_velocity = -robotVelocity.vyMetersPerSecond;
        Translation3d predicted_target_location = currentTarget.get().getTranslation();

        for (int i = 0; i < convergence_trials; i++) {
            // Gets current distance to target
            double distance = predicted_target_location.toTranslation2d()
                    .getDistance(robotPose.get().getTranslation().toTranslation2d());
            // Pivot Angle
            double pivotAngle = interpolatePivotAngle(distance);
            double shooterAngle = getShooterAngle(pivotAngle);
            double horizontalVelocity = launchVelocity * Math.cos(Units.degreesToRadians(shooterAngle));

            // TODO: Possibly account for pivot offset and shooter exit point

            // Get time to target
            double time = distance / horizontalVelocity;
            // apply time to target to speaker position
            predicted_target_location = new Translation3d(currentTarget.get().getX() + speaker_x_velocity * time,
                    currentTarget.get().getY() + speaker_y_velocity * time, currentTarget.get().getZ());

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

    public double getFlatDistanceToTarget() {
        return currentTarget.get().getTranslation().toTranslation2d()
                .getDistance(robotPose.get().getTranslation().toTranslation2d());
    }

    public double getHorizontalAngle(double launchVelocity) {
        ChassisSpeeds speeds = swerve.getChassisSpeeds();
        // Extrapolate target position based on robot velocity
        extrapolateTargetPosition(launchVelocity, speeds, robotPose.get());
        double angle = Math.atan2(extrapolatedTarget.getY() - robotPose.get().getTranslation().getY(),
                extrapolatedTarget.getX() - robotPose.get().getTranslation().getX());

        values.set(HORIZONTAL_ANGLE, angle);
        return Units.radiansToDegrees(angle);
    }

    public Optional<Rotation2d> getRotationTarget(double launchVelocity) {
        return Optional.of(Rotation2d.fromDegrees(getHorizontalAngle(launchVelocity)));
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
        double distance = target.getTranslation().toTranslation2d()
                .getDistance(robot.getTranslation().toTranslation2d());
        if (distance > 4.25)
            distance = 4.75;
        return pivotInterpolation
                .get(distance);
    }

    public double getYawToTarget(Pose3d robot){
        double x = currentTarget.get().getTranslation().getX() - robot.getTranslation().getX();
        double y = currentTarget.get().getTranslation().getY() - robot.getTranslation().getY();
        double static_angle = Math.atan2(y, x);

        return Units.radiansToDegrees(static_angle);

    }

    public double getTurnPower() {
        double targetAngle = getHorizontalAngle();
        updateControllerValues();
        double output = 0;
        if (shooterCamera.isValidTarget()) {
            double angle = shooterCamera.getAngleX();
            output = angle * -0.002;
        } else {
            thetaController.setSetpoint(Units.degreesToRadians(targetAngle));
            output = thetaController.calculate(robotPose.get().getRotation().getZ());
        }
        SmartDashboard.putBoolean("Theta Controller at Setpoint", thetaController.atSetpoint());
        return thetaController.atSetpoint() ? 0.0
                : output
                        * swerve.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();
        // Stops rotating robot once at setpoint within tolerance.
    }

    public void periodic() {
        ranExtrapolation = false;
    }

}
