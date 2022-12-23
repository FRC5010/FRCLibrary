// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Impl;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FRC5010.DrivetrainPoseEstimator;
import frc.robot.FRC5010.GenericDrivetrain;

/** Add your docs here. */
public class SimulatedDrivetrain extends GenericDrivetrain {
    DrivetrainPoseEstimator poseEstimator;
    public SimulatedDrivetrain(DrivetrainPoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        Pose2d pose = poseEstimator.getCurrentPose();
        Transform2d direction = new Transform2d(
            new Translation2d(chassisSpeeds.vxMetersPerSecond * 0.02, chassisSpeeds.vyMetersPerSecond * 0.02), 
            new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * 0.02));
        pose = pose.transformBy(direction);
        poseEstimator.resetToPose(pose);
    }

    public Rotation2d getHeading() {
        return poseEstimator.getGyroRotation2d();
    }
}
