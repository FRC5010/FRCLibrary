// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.swerve;

import java.util.HashMap;

import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.drive.GenericDrivetrain;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.pose.SwervePose;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;

/** Add your docs here. */
public class SdsSwerveDrivetrain extends GenericDrivetrain {
    private ChassisSpeeds chassisSpeeds;

    private GenericSwerveModule frontLeft, frontRight, backLeft, backRight;

    private SwerveConstants swerveConstants;

    public static final double MAX_VOLTAGE = 12.0;

    public SdsSwerveDrivetrain(Mechanism2d mechVisual,
            GenericSwerveModule frontLeft, GenericSwerveModule frontRight, GenericSwerveModule backLeft,
            GenericSwerveModule backRight,
            GenericGyro gyro, SwerveConstants constants,
            VisionSystem visionSystem) {
        super(mechVisual);

        this.swerveConstants = constants;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        poseEstimator = new DrivetrainPoseEstimator(
                new SwervePose(gyro, swerveConstants.getKinematics(), frontLeft, frontRight, backLeft, backRight),
                visionSystem);

        setDrivetrainPoseEstimator(poseEstimator);

        gyro.reset();
    }

    @Override
    public void drive(ChassisSpeeds direction) {
        chassisSpeeds = direction;
        SwerveModuleState[] states = swerveConstants.getKinematics().toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] setDesiredStates) {
        SwerveModuleState[] states = setDesiredStates;
        SwerveDriveKinematics.desaturateWheelSpeeds(states, swerveConstants.getkPhysicalMaxSpeedMetersPerSecond());
        
        frontLeft.setState(states[0], true);
        frontRight.setState(states[1], true);
        backLeft.setState(states[2], true);
        backRight.setState(states[3], true);
    }

    @Override
    public BaseAutoBuilder setAutoBuilder(HashMap<String, Command> eventMap) {
        return new SwerveAutoBuilder(
            () -> getPoseEstimator().getCurrentPose(), // Pose2d supplier
            (Pose2d pose) -> getPoseEstimator().resetToPose(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
            swerveConstants.getKinematics(), // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0),// PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            this::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // The drive subsystem. Used to properly set the requirements of path following commands
        );        
    }

}
