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
        values.declare(kP, 0.28);
        values.declare(kI, 0.0);
        values.declare(kD, 0.04);
        values.declare(TOLERANCE, 0.01);

        values.declare(TURN_POWER, 0.0);
        values.declare(HORIZONTAL_ANGLE, 0.0);
        values.declare(PIVOT_ANGLE, 0.0);
    

        // Initialize the PID controller
        thetaController = new PIDController(values.getDouble(kP), values.getDouble(kI), values.getDouble(kD));
        thetaController.setTolerance(values.getDouble(TOLERANCE));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // pivotInterpolation.put(1.31, -9.13);
        // pivotInterpolation.put(1.5048895591176392, -7.03);
        // pivotInterpolation.put(2.03, 0.70);
        // pivotInterpolation.put(2.51, 11.97);
        // pivotInterpolation.put(3.0603081350196564, 16.15);
        // pivotInterpolation.put(3.5087656150916264, 17.23);
        // pivotInterpolation.put(4.0007891303420235, 20.0);
        // pivotInterpolation.put(4.506227107522188, 22.48);
        // pivotInterpolation.put(5.000167775843761, 22.0);

        // pivotInterpolation.put(1.32, -9.13);
        // pivotInterpolation.put(1.51, -7.03);
        // pivotInterpolation.put(1.76, -2.80);
        // pivotInterpolation.put(2.01, 3.52);
        // pivotInterpolation.put(2.26, 5.29);
        // pivotInterpolation.put(2.52, 8.79);
        // pivotInterpolation.put(2.75, 11.24);
        // pivotInterpolation.put(3.00, 11.96);
        // pivotInterpolation.put(3.26, 14.0);
        // pivotInterpolation.put(3.53, 15.80);
        // pivotInterpolation.put(3.75, 16.87);
        // pivotInterpolation.put(4.00, 18.26);
        // pivotInterpolation.put(4.25, 19.33);
        // pivotInterpolation.put(4.49, 20.37);
        // pivotInterpolation.put(4.76, 20.40);

        pivotInterpolation.put(1.01, -11.0);
        pivotInterpolation.put(1.25, -2.5);
        pivotInterpolation.put(1.50, 2.46);
        pivotInterpolation.put(1.76, 5.46);
        pivotInterpolation.put(2.00, 7.46);
        pivotInterpolation.put(2.25, 9.46);
        pivotInterpolation.put(2.51, 12.46);
        pivotInterpolation.put(2.75, 14.46);
        pivotInterpolation.put(3.00, 16.96);
        pivotInterpolation.put(3.25, 16.96);
        pivotInterpolation.put(3.48, 18.46);
        pivotInterpolation.put(3.77, 19.73);
        pivotInterpolation.put(4.00, 18.26);
        pivotInterpolation.put(4.25, 19.33);
        pivotInterpolation.put(4.49, 20.37);
        pivotInterpolation.put(4.76, 20.40);
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
        return currentTarget.get().getTranslation().toTranslation2d().getDistance(robotPose.get().getTranslation().toTranslation2d());
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
        return pivotInterpolation
                .get(target.getTranslation().toTranslation2d().getDistance(robot.getTranslation().toTranslation2d()));
    }

    public double getTurnPower() {
        double targetAngle = getHorizontalAngle();
        updateControllerValues();
        thetaController.setSetpoint(Units.degreesToRadians(targetAngle));
        double output = thetaController.calculate(robotPose.get().getRotation().getZ());
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
