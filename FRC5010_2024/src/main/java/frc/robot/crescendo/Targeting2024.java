// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import java.util.function.Supplier;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;

/** Add your docs here. */
public class Targeting2024 {

    private static PIDController thetaController;
    private TrapezoidProfile.Constraints thetaConstraints;
    private SwerveDrivetrain swerve;
    private GenericPID thetaPID = new GenericPID(0.25, 0, 0.005);

    Supplier<Pose3d> robotPose;
    Supplier<Pose3d> targetPose;

    public Targeting2024(SwerveDrivetrain swerve, Supplier<Pose3d> robotPose, Supplier<Pose3d> targetPosition) {
        thetaConstraints = new TrapezoidProfile.Constraints(
                swerve.getSwerveConstants().getkPhysicalMaxAngularSpeedRadiansPerSecond(),
                swerve.getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond());

        thetaController = new PIDController(thetaPID.getkP(), thetaPID.getkI(), thetaPID.getkD());
        thetaController.setTolerance(0.05);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.targetPose = targetPosition;
        this.robotPose = robotPose;
        this.swerve = swerve;
    }

    public void init() {
        thetaController.reset();
    }

    public double getAnglePowerToTarget(ChassisSpeeds chassisSpeeds) {

        double targetAngle = AccountForHorizontalVelocity(robotPose.get(), Constants.Field.SHOT_POSE,
                chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
                getPivotAngleToTarget(robotPose.get(), Constants.Field.SHOT_POSE), 50);
        SmartDashboard.putNumber("Yaw Angle to Target", targetAngle);
        thetaController.setSetpoint(Units.degreesToRadians(targetAngle));
        return thetaController.atSetpoint() ? 0 : thetaController.calculate(robotPose.get().getRotation().getZ())
                * swerve.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();
    }

    public static double getYawAngleToTarget(Pose3d from, Pose3d target) {
        Translation3d difference = target.getTranslation().minus(from.getTranslation());
        double yawAngle = Math.atan2(difference.getY(), difference.getX());
        // yawAngle += Math.PI / 2;
        yawAngle = Math.toDegrees(yawAngle);
        return yawAngle;
    }

    public static double AccountForHorizontalVelocity(Pose3d robotPosition, Pose3d speakerPosition,
            double robotXVelocity, double robotYVelocity, double robotPivotAngle, double noteVelocity) {
        double noteVel = noteVelocity;
        double xDifference = speakerPosition.getY() - robotPosition.getY(); // In respect to speaker -(robotY -
                                                                            // speakerY)
        double yDifference = robotPosition.getX() - speakerPosition.getX(); // In respect to speaker (must be positive)
                                                                            // (robotX - speakerX)
        SmartDashboard.putNumber("x-difference", yDifference);
        SmartDashboard.putNumber("y-difference", -xDifference);
        double robotXVel = robotYVelocity; // In respect to speaker (robotYVelocity)
        double robotYVel = -robotXVelocity; // In respect to speaker -(robotXVelocity)
        double pivotAngle = Math.toRadians(robotPivotAngle);
        double targetValue = ((xDifference * robotXVel) - (yDifference * robotYVel)) / (noteVel * Math.cos(pivotAngle));
        double currentTheta = xDifference > 0.0 ? -Math.PI / 4 : Math.PI / 4;
        double currentDifference = (yDifference * Math.cos(currentTheta)) - (xDifference * Math.sin(currentTheta))
                - targetValue;
        double currentInterval = Math.PI / 2;

        while (currentInterval > 0.002) {
            currentTheta += Math.signum(currentDifference) * currentInterval;
            currentInterval /= 2;
            currentDifference = (yDifference * Math.cos(currentTheta)) - (xDifference * Math.sin(currentTheta))
                    - targetValue;
        }

        return Math.toDegrees(currentTheta) + 90 > 180 ? Math.toDegrees(currentTheta) - 270
                : Math.toDegrees(currentTheta) + 90;
    }

    public static double getPivotAngleToTarget(Pose3d from, Pose3d target) { // Change to actually work with weird
                                                                             // pivot...
        Translation3d difference = target.getTranslation().minus(from.getTranslation());
        double hypotenousA = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));
        double pivotAngle = Math.atan2(difference.getZ(), hypotenousA);
        return Math.toDegrees(pivotAngle);
    }

    public double getTargetingRotation(Supplier<Translation3d> robotTranslation, SwerveDrivetrain drivetrain) {
        // TODO: Have to account for robot rotation just like with camera
        
        ChassisSpeeds robotVelocity = drivetrain.getChassisSpeeds();
        robotVelocity = robotVelocity != null ? robotVelocity : new ChassisSpeeds();
        double angleSpeed = getAnglePowerToTarget(robotVelocity);
        SmartDashboard.putNumber("Targeting Angle Power", angleSpeed);
        return angleSpeed;
    }

}
