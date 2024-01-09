// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.commands.DefaultDriveCommand;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.sensors.Controller;

/** Add your docs here. */
public abstract class GenericDrivetrain extends SubsystemBase {
    protected Mechanism2d mechVisual;
    protected DrivetrainPoseEstimator poseEstimator;
    protected boolean isFieldOrientedDrive = true;

    public GenericDrivetrain(Mechanism2d mechVisual) {
        this.mechVisual = mechVisual;
        Shuffleboard.getTab("Drive").addBoolean("Field Oriented", () -> isFieldOrientedDrive).withPosition(8, 0);
    }

    public void setDrivetrainPoseEstimator(DrivetrainPoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public DrivetrainPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public Mechanism2d getMechVisual() {
        assert (null != poseEstimator);
        return mechVisual;
    }

    public Rotation2d getHeading() {
        assert (null != poseEstimator);
        return poseEstimator.getGyroRotation2d();
    };

    public abstract void drive(ChassisSpeeds direction);

    @Override
    public void periodic() {
        poseEstimator.update();
        // lockWheels();
    }

    public abstract void setAutoBuilder();

    public void disabledBehavior() {
    }

    public void toggleFieldOrientedDrive() {
        isFieldOrientedDrive = !isFieldOrientedDrive;
    }

    public void resetOrientation() {
        poseEstimator.resetToPose(new Pose2d(poseEstimator.getCurrentPose().getTranslation(), new Rotation2d()));
    }

    public void lockWheels() {
    }

    public Command createDefaultCommand(Controller driver) {
        return new DefaultDriveCommand(this,
                () -> driver.getLeftYAxis(),
                () -> driver.getLeftXAxis(),
                () -> driver.getRightXAxis(),
                () -> isFieldOrientedDrive);
    }

    public boolean hasIssues() {
        return false;
    }

    public void resetEncoders() {
    }

}
