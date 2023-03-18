// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.pose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FRC5010.drive.swerve.YAGSLSwerveDrivetrain;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import swervelib.math.SwerveKinematics2;

/** Add your docs here. */
public class YAGSLSwervePose extends GenericPose {
    private SwerveDrivePoseEstimator poseEstimator; 
    private YAGSLSwerveDrivetrain drivetrain; 


    public YAGSLSwervePose(GenericGyro gyro, SwerveKinematics2 kinematics, YAGSLSwerveDrivetrain drivetrain){
        super(gyro); 
 
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation2d(), drivetrain.getModulePositions(), new Pose2d());
    }
    
    @Override
    public void resetEncoders() {
        drivetrain.resetEncoders();
    }

    @Override
    public void updateVisionMeasurements(Pose2d robotPose, double imageCaptureTime) {
        poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
        
    }

    @Override
    public void updateLocalMeasurements() {
        poseEstimator.update(getGyroRotation2d(), drivetrain.getModulePositions());
        
    }

    @Override
    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void resetToPose(Pose2d pose) {
        poseEstimator.resetPosition(pose.getRotation(), drivetrain.getModulePositions(), pose);
        gyro.setAngle(pose.getRotation().getDegrees());
    }

    
}
