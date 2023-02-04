// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import java.util.HashMap;

import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.constants.RobotConstantsDef;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.pose.SimulatedPose;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;

/** Add your docs here. */
public class SimulatedDrivetrain extends GenericDrivetrain {
    private MechanismRoot2d unicycle;
    private MechanismLigament2d wheel;
    private Persisted<Integer> driveVisualH;
    private Persisted<Integer> driveVisualV;

    public SimulatedDrivetrain(GenericGyro gyro, VisionSystem vision, Mechanism2d mechVisual) {
        super(mechVisual);
        setDrivetrainPoseEstimator(new DrivetrainPoseEstimator(new SimulatedPose(gyro), vision));

        driveVisualH = new Persisted<>(RobotConstantsDef.DRIVE_VISUAL_H, Integer.class);
        driveVisualV = new Persisted<>(RobotConstantsDef.DRIVE_VISUAL_V, Integer.class);
    
        Integer centerH = driveVisualH.getInteger() / 2;
        Integer centerV = driveVisualV.getInteger() / 2;

        unicycle = mechVisual.getRoot("unicycle", centerH, centerV);
        wheel = new MechanismLigament2d("wheel", 1.0, 0.0, 6.0, new Color8Bit(Color.kYellow));
        unicycle.append(wheel);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        Pose2d pose = poseEstimator.getCurrentPose();
        Transform2d direction = new Transform2d(
            new Translation2d(chassisSpeeds.vxMetersPerSecond * 0.02, chassisSpeeds.vyMetersPerSecond * 0.02), 
            new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * 0.02));
        pose = pose.transformBy(direction);
        poseEstimator.resetToPose(pose);

        // Update visualizaton
        wheel.setAngle(pose.getRotation());
        wheel.setLength(direction.getTranslation().getNorm() * 1500);
    }

    public Rotation2d getHeading() {
        return poseEstimator.getGyroRotation2d();
    }

    @Override
    public BaseAutoBuilder setAutoBuilder(HashMap<String, Command> eventMap) {
        // TODO Auto-generated method stub
        return null;
    }
}
