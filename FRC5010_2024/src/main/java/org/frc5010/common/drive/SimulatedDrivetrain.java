// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.frc5010.common.arch.Persisted;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.drive.pose.DrivetrainPoseEstimator;
import org.frc5010.common.drive.pose.SimulatedPose;
import org.frc5010.common.sensors.gyro.GenericGyro;
import org.frc5010.common.vision.VisionSystem;

/** Add your docs here. */
public class SimulatedDrivetrain extends GenericDrivetrain {
  private MechanismRoot2d unicycle;
  private MechanismLigament2d wheel;
  private Persisted<Integer> driveVisualH;
  private Persisted<Integer> driveVisualV;
  private ChassisSpeeds chassisSpeeds;

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
    this.chassisSpeeds = chassisSpeeds;
    Pose2d pose = poseEstimator.getCurrentPose();
    Transform2d direction =
        new Transform2d(
            new Translation2d(
                chassisSpeeds.vxMetersPerSecond * 0.02, chassisSpeeds.vyMetersPerSecond * 0.02),
            new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * 0.02));
    pose = pose.transformBy(direction);
    poseEstimator.resetToPose(pose);

    // Update visualizaton
    wheel.setAngle(pose.getRotation());
    wheel.setLength(direction.getTranslation().getNorm() * 1500);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  public Rotation2d getHeading() {
    return poseEstimator.getGyroRotation2d();
  }

  @Override
  public void setAutoBuilder() {
    AutoBuilder.configureRamsete(
        () -> getPoseEstimator().getCurrentPose(), // Pose2d supplier
        (Pose2d pose) -> getPoseEstimator().resetToPose(pose),
        this::getChassisSpeeds, // Current ChassisSpeeds supplier
        this::drive, // Method that will drive the robot given ChassisSpeeds
        new ReplanningConfig(), // Default path replanning config. See the API for the options here
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }
}
