// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.FRC5010.Vision.VisionLimeLightSim;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.DrivePorts;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.robots.RobotFactory.Parts;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.NavXGyro;

/** Add your docs here. */
public class DefaultRobot extends RobotConfig {
    public DefaultRobot() {
        VisionSystem vision = new VisionLimeLightSim("Vision", 1);
        GenericGyro gyro = new NavXGyro(SPI.Port.kMXP);

        List<DrivePorts> motorPorts = new ArrayList<>();

        GenericMechanism drive = new Drive(vision, gyro, Drive.Type.DIFF_DRIVE, motorPorts);
        robotParts.put(Parts.VISION, vision);
        robotParts.put(Parts.DRIVE, drive);

    }
}
