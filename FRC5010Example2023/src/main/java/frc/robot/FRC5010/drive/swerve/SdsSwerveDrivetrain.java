// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.swerve;

import java.util.HashMap;

import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.drive.GenericDrivetrain;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.pose.SwervePose;
import frc.robot.FRC5010.mechanisms.DriveConstantsDef;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;

/** Add your docs here. */
public class SdsSwerveDrivetrain extends GenericDrivetrain {
    private ChassisSpeeds chassisSpeeds;

    private GenericGyro gyro; 

    private GenericSwerveModule frontLeft, frontRight, backLeft, backRight;

    private SwerveConstants swerveConstants;

    private Persisted<Double> maxChassisVelocity;

    public SdsSwerveDrivetrain(Mechanism2d mechVisual, GenericGyro gyro, SwerveConstants constants, VisionSystem visionSystem) {
        super(mechVisual);
        this.swerveConstants = swerveConstants;

        this.gyro = gyro;
        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        poseEstimator = new DrivetrainPoseEstimator(new SwervePose(gyro, swerveConstants.getKinematics(), frontLeft, frontRight, backLeft, backRight), visionSystem);
        maxChassisVelocity = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_VELOCITY, Double.class);

        setDrivetrainPoseEstimator(poseEstimator);

        gyro.reset();          
    }

    @Override
    public void drive(ChassisSpeeds direction) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public BaseAutoBuilder setAutoBuilder(HashMap<String, Command> eventMap) {
        // TODO Auto-generated method stub
        return null;
    }

}
