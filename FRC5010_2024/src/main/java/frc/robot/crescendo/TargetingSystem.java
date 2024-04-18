// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;

/** Add your docs here. */
public class TargetingSystem extends GenericSubsystem {
    private Supplier<Pose3d> currentTarget;
    // Velocity Extrapolation
    private Pose3d extrapolatedTarget = new Pose3d();
    private int convergence_trials = 10;
    private boolean ranExtrapolation = false;
    private double targetAngle, psuedoTargetAngle;

    private Supplier<Pose3d> robotPose;
    private SwerveDrivetrain swerve;
    private VisionSystem shooterCamera;
    private boolean useShooterCamera = false;
    private FeederSubsystem feeder;
    private GenericGyro gyro;

    private PIDController thetaController;
    private PIDController thetaCameraController;

    // Input/Output Values
    private final String kP = "Targeting kP";
    private final String kI = "Targeting kI";
    private final String kD = "Targeting kD";
    private final String KI_ZONE = "Targeting KI Zone";
    private final String TOLERANCE = "Targeting Tolerance";
    private final String MOVING_TOLERANCE = "Moving Targeting Tolerance";

    private static InterpolatingDoubleTreeMap pivotInterpolation = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap shuttlePivotInterpolation = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap shuttleShooterInterpolation = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap shooterInterpolation = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap yawDeficitInterpolation = new InterpolatingDoubleTreeMap();

    private final static double SHUTTLE_X_DISTANCE = 5.5;

    // Output Values
    private final String TURN_POWER = "Turn Power";
    private final String HORIZONTAL_ANGLE = "Horizontal Angle";
    private final String PIVOT_ANGLE = "Pivot Angle";

    private final double DEFAULT_TOLERANCE = 0.02;
    private final double SHOOTER_ANGLE_OFFSET = 0.0;
    private final double DEFAULT_MOVING_TOLERANCE = 0.04;
    private final double INITIAL_TIME_GUESS = 0.5;
    private final double MINIMUM_VELOCITY_THRESHOLD = 0.1;
    private boolean accountForMovement = false;
    private boolean interpolatingYaw = false;

    public TargetingSystem(Supplier<Pose3d> targetSupplier, Supplier<Pose3d> robotPose, SwerveDrivetrain swerve,
            VisionSystem shooter, FeederSubsystem feeder, GenericGyro gyro) {
        this.currentTarget = targetSupplier;
        this.robotPose = robotPose;
        this.swerve = swerve;
        this.shooterCamera = shooter;
        this.gyro = gyro;
        this.feeder = feeder;

        // Declare Values
        values.declare(kP, 0.11);
        values.declare(kI, 0.0);
        values.declare(KI_ZONE, 0.00);
        values.declare(kD, 0.0015);
        values.declare(TOLERANCE, DEFAULT_TOLERANCE);
        values.declare(MOVING_TOLERANCE, DEFAULT_MOVING_TOLERANCE);

        values.declare(TURN_POWER, 0.0);
        values.declare(HORIZONTAL_ANGLE, 0.0);
        values.declare(PIVOT_ANGLE, 0.0);

        // Initialize the PID controller
        thetaController = new PIDController(values.getDouble(kP), values.getDouble(kI), values.getDouble(kD));
        thetaController.setIZone(values.getDouble(KI_ZONE));
        thetaController.setTolerance(values.getDouble(TOLERANCE));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        thetaCameraController = new PIDController(values.getDouble(kP), values.getDouble(kI), values.getDouble(kD));
        thetaCameraController.setIZone(values.getDouble(KI_ZONE));
        thetaCameraController.setTolerance(values.getDouble(TOLERANCE));
        thetaCameraController.enableContinuousInput(-Math.PI, Math.PI);
        thetaCameraController.setSetpoint(0.0);

        // pivotInterpolation.put(0.5, -11.0);
        // pivotInterpolation.put(1.25, -10.51);
        // pivotInterpolation.put(1.50, -4.59);
        // pivotInterpolation.put(1.75, 1.11);
        // pivotInterpolation.put(2.06, 7.19);
        // pivotInterpolation.put(2.25, 7.43);
        // pivotInterpolation.put(2.50, 11.44);
        // pivotInterpolation.put(2.75, 14.94);
        // pivotInterpolation.put(3.03, 17.44);
        // pivotInterpolation.put(3.25, 17.69);
        // pivotInterpolation.put(3.50, 22.56);
        // pivotInterpolation.put(3.70, 25.25);
        // pivotInterpolation.put(4.06, 26.14);// 25.91
        // pivotInterpolation.put(4.26, 26.41);
        // pivotInterpolation.put(4.50, 27.31);
        // pivotInterpolation.put(4.75, 28.31);
        // pivotInterpolation.put(4.94, 28.81);

        pivotInterpolation.put(1.36, -9.8);
        pivotInterpolation.put(1.498285686, -8.036407471);
        pivotInterpolation.put(1.766461413, -0.198120117);
        pivotInterpolation.put(2.006095159, 4.50677371);
        pivotInterpolation.put(2.253810077, 6.185724354);
        pivotInterpolation.put(2.498182415, 10.58572435);

        pivotInterpolation.put(2.739574022, 13.66646194);
        pivotInterpolation.put(3.000037528, 15.20387955);
        pivotInterpolation.put(3.249798442, 18.77407951);
        pivotInterpolation.put(3.504684604, 20.67407951);
        pivotInterpolation.put(3.765572428, 23.18393135);
        pivotInterpolation.put(3.991071211, 24.24553108);
        pivotInterpolation.put(4.241035167, 25.89009056);// 25.91
        pivotInterpolation.put(4.494075064, 27.17321701);
        pivotInterpolation.put(4.752376138, 28.12282753);
        pivotInterpolation.put(5.00374855, 28.88491821);

        shuttlePivotInterpolation.put(5.1, -10.0);
        shuttlePivotInterpolation.put(15.0, 0.0);

        shooterInterpolation.put(1.0, Constants.Physical.SUBWOOFER_SHOT);
        shooterInterpolation.put(5.0, Constants.Physical.TOP_SHOOTING_SPEED);
        shuttleShooterInterpolation.put(5.1, Constants.Physical.SHUTTLE_SPEED); // Delete
        shuttleShooterInterpolation.put(12.0, Constants.Physical.SHUTTLE_SPEED_HIGH); // Delete

        // Interpolate yaw deficit here
        yawDeficitInterpolation.put(3.29315185546875, 0.289209397);
        yawDeficitInterpolation.put(54.68994140625, 2.077867226);
        yawDeficitInterpolation.put(9.89044189453125, 0.524646528);
        yawDeficitInterpolation.put(0.0, 0.0);
        yawDeficitInterpolation.put(0.0, 0.0);
        yawDeficitInterpolation.put(0.0, 0.0);
        yawDeficitInterpolation.put(0.0, 0.0);
        yawDeficitInterpolation.put(0.0, 0.0);
        yawDeficitInterpolation.put(0.0, 0.0);
        yawDeficitInterpolation.put(0.0, 0.0);
        yawDeficitInterpolation.put(0.0, 0.0);
        yawDeficitInterpolation.put(0.0, 0.0);
        yawDeficitInterpolation.put(0.0, 0.0);
        yawDeficitInterpolation.put(0.0, 0.0);
    }

    public static Pose3d getSpeakerTarget(Alliance alliance) {
        return alliance == Alliance.Blue ? Constants.Field.BLUE_SHOT_POSE : Constants.Field.RED_SHOT_POSE;
    }

    public static Pose3d getShuttleTarget(Alliance alliance) {
        return alliance == Alliance.Blue ? Constants.Field.BLUE_SHUTTLE_POSE : Constants.Field.RED_SHUTTLE_POSE;
    }

    public static Pose3d getDefaultTarget() {
        return getSpeakerTarget(Alliance.Blue);
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

    public void useShooterCamera(boolean use) {
        useShooterCamera = use;
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
        thetaController.setI(values.getDouble(KI_ZONE));
        thetaController.setTolerance(values.getDouble(TOLERANCE));

        thetaCameraController.setP(values.getDouble(kP));
        thetaCameraController.setI(values.getDouble(kI));
        thetaCameraController.setD(values.getDouble(kD));
        thetaCameraController.setI(values.getDouble(KI_ZONE));
        thetaCameraController.setTolerance(values.getDouble(TOLERANCE));
    }

    public boolean isAtTargetYaw() {
        if (accountForMovement && moving()
                && Math.abs(gyro.getAngle() - psuedoTargetAngle) < values.getDouble(MOVING_TOLERANCE)) {
            return true;
        } else if (accountForMovement && moving()) {
            return false;
        }
        return thetaController.atSetpoint();
    }

    public boolean isAtTargetCameraYaw() {
        return thetaCameraController.atSetpoint();
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

    public double getShooterSpeed() {
        double flatDistance = getFlatDistanceToTarget();
        if (Math.abs(robotPose.get().getX() - currentTarget.get().getX()) > SHUTTLE_X_DISTANCE) {
            return shuttleShooterInterpolation.get(flatDistance);
        }
        return shooterInterpolation.get(flatDistance);
    }

    public double getShooterSpeed(double distance) {
        return shooterInterpolation.get(distance);
    }

    public static double interpolateShooterSpeed(double distance) {
        return shooterInterpolation.get(distance);
    }

    public Optional<Double> getShooterCamAngle() {
        if (useShooterCamera && shooterCamera.isValidTarget()) {
            return Optional.of(interpolatePivotAngle(shooterCamera.getDistance()));
        }
        return Optional.empty();
    }

    public Optional<Double> getShooterYawPower() {
        // double turnFactor = -0.002;
        if (useShooterCamera && shooterCamera.isValidTarget()) {
            double angle = Units.degreesToRadians(shooterCamera.getAngleX() + SHOOTER_ANGLE_OFFSET);
            return Optional.of(thetaCameraController.calculate(angle));
        }
        return Optional.empty();
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

        SmartDashboard.putNumber("Horizontal Targeting Difference X:", x);
        SmartDashboard.putNumber("Horizontal Targeting Difference Y:", y);
        double static_angle = Math.atan2(y, x);
        SmartDashboard.putNumber("Horizontal Targeting Static Angle Radians:", static_angle);
        values.set(HORIZONTAL_ANGLE, Units.radiansToDegrees(static_angle));
        SmartDashboard.putNumber("Horizontal Targeting Static Angle Degrees", Units.radiansToDegrees(static_angle));
        return Units.radiansToDegrees(static_angle);
    }

    public double getFlatDistanceToTarget() {
        return currentTarget.get().getTranslation().toTranslation2d()
                .getDistance(robotPose.get().getTranslation().toTranslation2d());
    }

    public double getFlatDistanceToTarget(Pose2d pose) {
        return currentTarget.get().getTranslation().toTranslation2d()
                .getDistance(pose.getTranslation());
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

        if (Math.abs(robot.getX() - target.getX()) > SHUTTLE_X_DISTANCE) { // Shuttling
            return shuttlePivotInterpolation.get(distance);
        }
        return pivotInterpolation
                .get(distance);
    }

    public double getYawToTarget(Pose3d robot) {
        double x = currentTarget.get().getTranslation().getX() - robot.getTranslation().getX();
        double y = currentTarget.get().getTranslation().getY() - robot.getTranslation().getY();
        double static_angle = Math.atan2(y, x);

        return Units.radiansToDegrees(static_angle);

    }

    public double getTurnPower() {
        if (!interpolatingYaw) {
            if (accountForMovement && moving()) {
                psuedoTargetAngle = getFutureYawAngle();
                thetaController.setSetpoint(Units.degreesToRadians(psuedoTargetAngle));
            } else {
                targetAngle = getHorizontalAngle();
                thetaController.setSetpoint(Units.degreesToRadians(targetAngle));
            }
        }
        updateControllerValues();
        double output = 0;

        output = thetaController.calculate(robotPose.get().getRotation().getZ());
        output = MathUtil.clamp(output, -0.50, 0.50);

        SmartDashboard.putBoolean("Theta Controller at Setpoint", thetaController.atSetpoint());
        return output * swerve.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();
        // Stops rotating robot once at setpoint within tolerance.
    }

    public double getInitialStationaryTurnPower() {
        targetAngle = getHorizontalAngle();
        thetaController.setSetpoint(Units.degreesToRadians(targetAngle));
        updateControllerValues();
        double output = 0;
        output = thetaController.calculate(robotPose.get().getRotation().getZ());
        output = MathUtil.clamp(output, -0.50, 0.50);
        SmartDashboard.putBoolean("Theta Controller at Setpoint", thetaController.atSetpoint());
        return output * swerve.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();
        // Stops rotating robot once at setpoint within tolerance.
    }

    public double getFutureYawAngle() {
        double time = INITIAL_TIME_GUESS;
        double angle = 360;
        double robotX, robotY, x, y;
        while (Math.abs(time - yawDeficitInterpolation.get(angle)) > 0.05 || angle == 360) {
            robotX = robotPose.get().getX() + swerve.getChassisSpeeds().vxMetersPerSecond * time
                    + 0.5 * ((Pigeon2) ((PigeonGyro) gyro).getGyro()).getAccelerationX().getValueAsDouble() * time
                            * time;
            robotY = robotPose.get().getY() + swerve.getChassisSpeeds().vyMetersPerSecond * time
                    + 0.5 * ((Pigeon2) ((PigeonGyro) gyro).getGyro()).getAccelerationY().getValueAsDouble() * time
                            * time;
            x = currentTarget.get().getTranslation().getX() - robotX;
            y = currentTarget.get().getTranslation().getY() - robotY;
            angle = Units.radiansToDegrees(Math.atan2(x, y));
            time = yawDeficitInterpolation.get(Math.abs(gyro.getAngle() - angle));
            SmartDashboard.putNumber("Horizontal Targeting Difference X:", x);
            SmartDashboard.putNumber("Horizontal Targeting Difference Y:", y);
        }
        return angle;
    }

    public boolean moving() {
        if (swerve.getChassisSpeeds().vxMetersPerSecond > MINIMUM_VELOCITY_THRESHOLD
                || swerve.getChassisSpeeds().vyMetersPerSecond > MINIMUM_VELOCITY_THRESHOLD) {
            return true;
        }
        return false;
    }

    public void setAccountForMovement(boolean bool) {
        this.accountForMovement = bool;
    }

    public void setInterpolatingYaw(boolean bool) {
        this.interpolatingYaw = bool;
    }

    public Pose3d getCurrentTarget() {
        return currentTarget.get();
    }
    public void periodic() {
        feeder.setShooterHasTarget(shooterCamera.isValidTarget());
        ranExtrapolation = false;
        SmartDashboard.putNumber("Targeting Pose X", currentTarget.get().getX());
        SmartDashboard.putNumber("Targeting Pose Y", currentTarget.get().getY());
        SmartDashboard.putNumber("Robot Targeting Pose X", currentTarget.get().getX());
        SmartDashboard.putNumber("Robot Targeting Pose Y", robotPose.get().getY());

    }

}
