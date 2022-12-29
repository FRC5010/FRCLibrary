// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.pose;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FRC5010.sensors.GenericEncoder;
import frc.robot.FRC5010.sensors.GenericGyro;

/** Add your docs here. */
public class DifferentialPose extends GenericPose {
    private GenericEncoder leftEncoder;
    private GenericEncoder rightEncoder;
    private final DifferentialDrivePoseEstimator poseEstimator;

    public DifferentialPose(GenericGyro gyro, GenericEncoder leftEncoder, GenericEncoder rightEncoder) {
        super(gyro);
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        poseEstimator = new DifferentialDrivePoseEstimator(
                getGyroRotation2d(),
                new Pose2d(),
                stateStdDevs,
                localMeasurementStdDevs,
                visionMeasurementStdDevs);
    }


    @Override
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void updateVisionMeasurements(Pose2d robotPose, double imageCaptureTime) {
        //poseEstimator.resetPosition(robotPose, robotPose.getRotation());
        // m_poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
    }

    public void updateLocalMeasurements() {
        DifferentialDriveWheelSpeeds actWheelSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
        poseEstimator.update(getGyroRotation2d(), actWheelSpeeds, 0, 0);

        actWheelSpeeds = new DifferentialDriveWheelSpeeds(
                leftEncoder.getVelocity(), rightEncoder.getVelocity());
        double leftDist = leftEncoder.getPosition();
        double rightDist = rightEncoder.getPosition();
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation2d(),
                actWheelSpeeds, leftDist, rightDist);
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    public void resetToPose(Pose2d pose) {
        poseEstimator.resetPosition(pose, getGyroRotation2d());
    }
}
