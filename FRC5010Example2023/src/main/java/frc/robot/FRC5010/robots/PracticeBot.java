// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.FRC5010.Vision.VisionLimeLightSim;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.robots.RobotFactory.Parts;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.NavXGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;

/** Add your docs here. */
public class PracticeBot extends RobotType {
    public PracticeBot() {
        VisionLimeLightSim multiVision = new VisionLimeLightSim("simulation", 0);
        // VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1, AprilTags.aprilTagRoomLayout,PoseStrategy.CLOSEST_TO_LAST_POSE);
        // multiVision.addPhotonCamera("Arducam_OV9281_USB_Camera", 
        //   new Transform3d( // This describes the vector between the camera lens to the robot center on the ground
        //     new Translation3d(Units.inchesToMeters(7), 0, Units.inchesToMeters(16.75)), 
        //     new Rotation3d(0, Units.degreesToRadians(-20), 0)
        //   )
        // );
        // multiVision.createRobotPoseEstimator();
        List<SwervePorts> swervePorts = new ArrayList<>();
        // TODO: Remap encoderPorts once PracticeBot has absolute encoders.
        swervePorts.add(new SwervePorts(1, 3, 0));
        swervePorts.add(new SwervePorts(10, 8, 1));
        swervePorts.add(new SwervePorts(5, 4, 2));
        swervePorts.add(new SwervePorts(2, 7, 3));

        GenericGyro gyro = new PigeonGyro(0); // TODO: Find CAN value.

        GenericMechanism drive = new Drive(multiVision, gyro, Drive.Type.MK4_SWERVE_DRIVE, swervePorts);
        robotParts.put(Parts.VISION, multiVision);
        robotParts.put(Parts.DRIVE, drive);
    }
}
