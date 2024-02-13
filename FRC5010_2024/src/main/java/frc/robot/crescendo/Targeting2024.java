// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;

/** Add your docs here. */
public class Targeting2024 {

    private static ProfiledPIDController thetaController;
    private TrapezoidProfile.Constraints thetaConstraints; 
    private SwerveDrivetrain swerve;
    private GenericPID thetaPID = new GenericPID(0.25, 0, 0);

    Supplier<Pose3d> robotPose;
    Supplier<Pose3d> targetPose;

    public Targeting2024(SwerveDrivetrain swerve, Supplier<Pose3d> robotPose, Supplier<Pose3d> targetPosition) {
        thetaConstraints = new TrapezoidProfile.Constraints(
            swerve.getSwerveConstants().getkPhysicalMaxAngularSpeedRadiansPerSecond(), swerve.getSwerveConstants().getkTeleDriveMaxAngularAccelerationUnitsPerSecond());

        thetaController = new ProfiledPIDController(thetaPID.getkP(), thetaPID.getkI(), thetaPID.getkD(), thetaConstraints);
        thetaController.setTolerance(0.01);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.targetPose = targetPosition;
        this.robotPose = robotPose;
        this.swerve = swerve;
    }   

    public void init() {
        thetaController.reset(robotPose.get().getRotation().getZ());
    }


    public double getAnglePowerToTarget() {
        double targetAngle = getYawAngleToTarget(robotPose.get(), targetPose.get());
        SmartDashboard.putNumber("Yaw Angle to Target", targetAngle);
        thetaController.setGoal(targetAngle);
        return thetaController.calculate(Units.radiansToDegrees(robotPose.get().getRotation().getZ())) * swerve.getSwerveConstants().getkTeleDriveMaxAngularSpeedRadiansPerSecond();
    }




    public static double getYawAngleToTarget(Pose3d from, Pose3d target) {
        Translation3d difference = target.getTranslation().minus(from.getTranslation());
        double yawAngle = Math.atan2(difference.getY(), difference.getX());
        // yawAngle += Math.PI / 2;
        yawAngle = Math.toDegrees(yawAngle);
        return yawAngle;
    }

    public static double getPivotAngleToTarget(Pose3d from, Pose3d target) { // Change to actually work with weird pivot...
        Translation3d difference = from.getTranslation().minus(target.getTranslation());
        double hypotenousA = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));
        double pivotAngle = Math.atan2(difference.getZ(), hypotenousA);
        return Math.toDegrees(pivotAngle);
    }






}
