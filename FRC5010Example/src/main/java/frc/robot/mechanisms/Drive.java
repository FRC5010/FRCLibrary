// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.Robot;
import frc.robot.FRC5010.Controller;
import frc.robot.FRC5010.GenericMechanism;
import frc.robot.FRC5010.VisionSystem;
import frc.robot.FRC5010.commands.DefaultDriveCommand;
import frc.robot.FRC5010.drive.DifferentialDrivetrain;
import frc.robot.FRC5010.drive.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.GenericDrivetrain;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.sensors.GenericGyro;
import frc.robot.FRC5010.sensors.SimulatedGyro;

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

    @Override
    protected void initRealOrSim() {
        /**
         * TODO: Add real implementation classes here
         */

        if (Robot.isReal()) {
            // TODO: replace gyro with a real one
            gyro = new SimulatedGyro();
        } else {
            gyro = new SimulatedGyro();
            //drivetrain = new SimulatedDrivetrain(gyro, vision, drivetrainVisual);
        }
        MotorController5010 template = MotorFactory.DriveTrainMotor(MotorFactory.NEO(1));
        List<Integer> motorPorts = new ArrayList<>();
        
        // This assumes ports 1 & 2 are left and 3 & 4 are right
        // This is just an example of how to put a sequence of numbers into a list
        motorPorts.addAll(IntStream.rangeClosed(1, 4).boxed().collect(Collectors.toList()));

        drivetrain = new DifferentialDrivetrain(template, motorPorts, gyro, vision, drivetrainVisual);
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
}
