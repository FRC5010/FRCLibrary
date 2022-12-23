// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.Robot;
import frc.robot.FRC5010.Controller;
import frc.robot.FRC5010.DrivetrainPoseEstimator;
import frc.robot.FRC5010.GenericDrivetrain;
import frc.robot.FRC5010.GenericGyro;
import frc.robot.FRC5010.GenericMechanism;
import frc.robot.FRC5010.GenericPose;
import frc.robot.FRC5010.VisionSystem;
import frc.robot.FRC5010.Impl.SimulatedDrivetrain;
import frc.robot.FRC5010.Impl.SimulatedGyro;
import frc.robot.FRC5010.Impl.SimulatedPose;
import frc.robot.FRC5010.commands.DefaultDriveCommand;

/** Add your docs here. */
public class Drive extends GenericMechanism {
    private VisionSystem vision;
    private Controller driver;
    DrivetrainPoseEstimator poseEstimator;
    GenericDrivetrain drivetrain;
    GenericGyro gyro;

    public Drive(Controller driver, VisionSystem visionSystem, Mechanism2d robotMechSim) {
        super(robotMechSim);
        this.driver = driver;
        this.vision = visionSystem;
        initRealOrSim();
    }

    public void setupDefaultCommands() {
        // Handle real or simulation case for default commands
        if (Robot.isReal()) {

        } else {
            
        }
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, 
            () -> driver.getLeftYAxis(), () -> driver.getLeftXAxis(), () -> driver.getRightXAxis()));
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        // If there needs to be some commands that are real or simulation only use this
        if (Robot.isReal()) {

        } else {
            
        }

        // Example of setting up axis for driving omnidirectional
        driver.setLeftXAxis(driver.createLeftXAxis()
            .negate().deadzone(0.07).limit(1).rate(2).cubed());
        driver.setLeftYAxis(driver.createLeftYAxis()
            .negate().deadzone(0.07).limit(1).rate(2).cubed());
        driver.setRightXAxis(driver.createRightXAxis()
            .negate().deadzone(0.07).limit(1).rate(4).cubed());
        // Put commands that can be both real and simulation afterwards
    }

    @Override
    protected void initRealOrSim() {
        GenericPose poseHandler = null;
        /**
         * TODO: Add real implentation classes here
         */
        if (Robot.isReal()) {
            // TODO: replace poseHandler with a real one
            poseHandler = new SimulatedPose(gyro);
        } else {
            gyro = new SimulatedGyro();
            poseHandler = new SimulatedPose(gyro);
            poseEstimator = new DrivetrainPoseEstimator(poseHandler, vision);
            drivetrain = new SimulatedDrivetrain(poseEstimator);
        }
    }
}
