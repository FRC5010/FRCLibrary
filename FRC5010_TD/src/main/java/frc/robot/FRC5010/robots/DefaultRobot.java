// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionLimeLightSim;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.DrivePorts;
import frc.robot.FRC5010.constants.GenericDrivetrainConstants;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.NavXGyro;

/** Add your docs here. */
public class DefaultRobot extends GenericMechanism {
    private GenericDrivetrainConstants driveConstants;
    private GenericMechanism drive;
    private VisionSystem vision;

    public DefaultRobot(Mechanism2d visual, ShuffleboardTab displayTab) {
        super(visual, displayTab);
        driveConstants = new GenericDrivetrainConstants();
        vision = new VisionLimeLightSim("Vision", 1, AprilTags.aprilTagRoomLayout);
        GenericGyro gyro = new NavXGyro(SPI.Port.kMXP);

        List<DrivePorts> motorPorts = new ArrayList<>();

        drive = new Drive(vision, gyro, Drive.Type.DIFF_DRIVE, motorPorts, driveConstants);
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {

    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        drive.setupDefaultCommands(driver, operator);
    }

    @Override
    protected void initRealOrSim() {
    }

    @Override
    public Map<String, List<PathPlannerTrajectory>> initAutoCommands() {
        return new HashMap<>();
    }

    @Override
    public Command generateAutoCommand(List<PathPlannerTrajectory> paths) {
        return drive.generateAutoCommand(paths);
    }
}
