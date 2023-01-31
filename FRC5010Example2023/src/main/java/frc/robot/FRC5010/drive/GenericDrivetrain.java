// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;

/** Add your docs here. */
public abstract class GenericDrivetrain extends SubsystemBase {
    protected Mechanism2d mechVisual;
    protected DrivetrainPoseEstimator poseEstimator;
    public GenericDrivetrain(Mechanism2d mechVisual) {
        this.mechVisual = mechVisual;
    }
    public void setDrivetrainPoseEstimator(DrivetrainPoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public DrivetrainPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }
    
    public Mechanism2d getMechVisual() { assert(null != poseEstimator); return mechVisual; }
    public Rotation2d getHeading() { assert(null != poseEstimator); return poseEstimator.getGyroRotation2d(); };

    public abstract void drive(ChassisSpeeds direction);

    @Override
    public void periodic() {
        poseEstimator.update();
    }
}
