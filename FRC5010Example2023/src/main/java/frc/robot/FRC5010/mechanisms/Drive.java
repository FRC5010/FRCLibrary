// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.mechanisms;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.commands.DefaultDriveCommand;
import frc.robot.FRC5010.constants.DrivePorts;
import frc.robot.FRC5010.constants.GenericSwerveModuleConstants;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.constants.RobotConstantsDef;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.drive.DifferentialDrivetrain;
import frc.robot.FRC5010.drive.GenericDrivetrain;
import frc.robot.FRC5010.drive.GenericSwerveModule;
import frc.robot.FRC5010.drive.MK4SwerveModule;
import frc.robot.FRC5010.drive.SwerveDrivetrain;
import frc.robot.FRC5010.drive.ThriftySwerveModule;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.commands.ChaseTag;

/** Add your docs here. */
public class Drive extends GenericMechanism {
    private VisionSystem vision;
    private GenericDrivetrain drivetrain;
    private GenericGyro gyro;
    private Command defaultDriveCommand;
    private String type;
    private List<? extends DrivePorts> motorPorts;

    public static class Type {
        public static final String DIFF_DRIVE = "DifferentialDrive";
        public static final String THRIFTY_SWERVE_DRIVE = "ThriftySwerveDrive";
        public static final String MK4_SWERVE_DRIVE = "MK4SwerveDrive";
        public static final String MK4I_SWERVE_DRIVE = "MK4ISwerveDrive";
    }

    // Examples of how to use a persisted constants
    // These can live in specific constants files, however
    private static Persisted<Integer> driveVisualH;
    private static Persisted<Integer> driveVisualV;

    public Drive(VisionSystem visionSystem, GenericGyro gyro, String type, List<? extends DrivePorts> drivePorts) {
        this.vision = visionSystem;
        this.gyro = gyro;
        this.type = type;
        motorPorts = drivePorts;
        driveVisualH = new Persisted<>(RobotConstantsDef.DRIVE_VISUAL_H, 60);
        driveVisualV = new Persisted<>(RobotConstantsDef.DRIVE_VISUAL_V, 60);
        mechVisual = new Mechanism2d(driveVisualH.getInteger(), driveVisualV.getInteger());

        initRealOrSim();
    }

    @Override
    protected void initRealOrSim() {
        switch(type) {
            case Type.DIFF_DRIVE: {
                initializeDifferentialDrive();
                break;
            }
            case Type.THRIFTY_SWERVE_DRIVE: {
                initializeThriftySwerveDrive();
                break;
            }
            case Type.MK4_SWERVE_DRIVE: {
                initializeMK4SwerveDrive();
                break;
            }
            case Type.MK4I_SWERVE_DRIVE: {
                break;
            }
            default: {
                break;
            }
        }
    }

    public void setupDefaultCommands() {
        // Handle real or simulation case for default commands
        if (Robot.isReal()) {

        } else {
            
        }
        drivetrain.setDefaultCommand(defaultDriveCommand);
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        // If there needs to be some commands that are real or simulation only use this
        if (Robot.isReal()) {

        } else {
            
        }
        driver.setLeftXAxis(driver.createLeftXAxis().deadzone(0.075).negate().rate(SwerveDrivetrain.kTeleDriveMaxAccelerationUnitsPerSecond));
        driver.setLeftYAxis(driver.createLeftYAxis().deadzone(0.075).negate().rate(SwerveDrivetrain.kTeleDriveMaxAccelerationUnitsPerSecond));
        driver.setRightXAxis(driver.createRightXAxis().deadzone(0.075).negate().rate(SwerveDrivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond));

        driver.createXButton().whileTrue(new ChaseTag(drivetrain, () -> drivetrain.getPoseEstimator().getCurrentPose(), vision));

        // Example of setting up axis for driving omnidirectional
        // driver.setLeftXAxis(driver.createLeftXAxis()
        //     .negate().deadzone(0.07).limit(1).rate(2).cubed());
        // driver.setLeftYAxis(driver.createLeftYAxis()
        //     .negate().deadzone(0.07).limit(1).rate(2).cubed());
        // driver.setRightXAxis(driver.createRightXAxis()
        //     .negate().deadzone(0.07).limit(1).rate(4).cubed());
        // Put commands that can be both real and simulation afterwards

        defaultDriveCommand = new DefaultDriveCommand(drivetrain, 
            () -> driver.getLeftYAxis(), 
            () -> driver.getLeftXAxis(), 
            () -> driver.getRightXAxis(),
            () -> driver.createAButton().getAsBoolean());
    }

    private void initializeThriftySwerveDrive() {
        GenericSwerveModuleConstants frontLeftConstants = new GenericSwerveModuleConstants(0, 0, false, 0, true, true);
        GenericSwerveModuleConstants frontRightConstants = new GenericSwerveModuleConstants(0, 0, false, 0, true, true);
        GenericSwerveModuleConstants backLeftConstants = new GenericSwerveModuleConstants(0, 0, true, 0, true, true);
        GenericSwerveModuleConstants backRightConstants = new GenericSwerveModuleConstants(0, 0, true, 0, true, true);

        GenericSwerveModule frontLeft = new ThriftySwerveModule(
            mechVisual.getRoot("frontleft", 45, 15), "frontleft", 
            SwerveDrivetrain.kFrontLeftAbsoluteOffsetRad, (SwervePorts)motorPorts.get(0), frontLeftConstants);  
        GenericSwerveModule frontRight = new ThriftySwerveModule(
            mechVisual.getRoot("frontright", 45, 45), "frontright", 
            SwerveDrivetrain.kFrontRightAbsoluteOffsetRad, (SwervePorts)motorPorts.get(1), frontRightConstants);
        GenericSwerveModule backLeft = new ThriftySwerveModule(
            mechVisual.getRoot("backleft", 15, 15), "backleft", 
            SwerveDrivetrain.kBackLeftAbsoluteOffsetRad, (SwervePorts)motorPorts.get(2), backLeftConstants);
        GenericSwerveModule backRight = new ThriftySwerveModule(
            mechVisual.getRoot("backright", 15, 45), "backright", 
            SwerveDrivetrain.kBackRightAbsoluteOffsetRad, (SwervePorts)motorPorts.get(3), backRightConstants);

        drivetrain = new SwerveDrivetrain(mechVisual, frontLeft, frontRight, backLeft, backRight, gyro, vision);
    }

    private void initializeDifferentialDrive() {
        MotorController5010 template = MotorFactory.DriveTrainMotor(MotorFactory.NEO(1));
        List<DrivePorts> motorPorts = new ArrayList<>();
        
        // This assumes ports 1 & 2 are left and 3 & 4 are right
        // This is just an example of how to put a sequence of numbers into a list
        motorPorts.add(new DrivePorts(1));
        motorPorts.add(new DrivePorts(2));
        motorPorts.add(new DrivePorts(3));
        motorPorts.add(new DrivePorts(4));

        drivetrain = new DifferentialDrivetrain(template, motorPorts, gyro, vision, mechVisual);
    }

    private void initializeMK4SwerveDrive() {
        GenericSwerveModuleConstants frontLeftConstants = new GenericSwerveModuleConstants(0, 0, false, 0, true, true);
        GenericSwerveModuleConstants frontRightConstants = new GenericSwerveModuleConstants(0, 0, false, 0, true, true);
        GenericSwerveModuleConstants backLeftConstants = new GenericSwerveModuleConstants(0, 0, true, 0, true, true);
        GenericSwerveModuleConstants backRightConstants = new GenericSwerveModuleConstants(0, 0, true, 0, true, true);

        GenericSwerveModule frontLeft = new MK4SwerveModule(
            mechVisual.getRoot("frontleft", 45, 15), "frontleft", 
            SwerveDrivetrain.kFrontLeftAbsoluteOffsetRad, (SwervePorts)motorPorts.get(0), frontLeftConstants);  
        GenericSwerveModule frontRight = new MK4SwerveModule(
            mechVisual.getRoot("frontright", 45, 45), "frontright", 
            SwerveDrivetrain.kFrontRightAbsoluteOffsetRad, (SwervePorts)motorPorts.get(1), frontRightConstants);
        GenericSwerveModule backLeft = new MK4SwerveModule(
            mechVisual.getRoot("backleft", 15, 15), "backleft", 
            SwerveDrivetrain.kBackLeftAbsoluteOffsetRad, (SwervePorts)motorPorts.get(2), backLeftConstants);
        GenericSwerveModule backRight = new MK4SwerveModule(
            mechVisual.getRoot("backright", 15, 45), "backright", 
            SwerveDrivetrain.kBackRightAbsoluteOffsetRad, (SwervePorts)motorPorts.get(3), backRightConstants);

        drivetrain = new SwerveDrivetrain(mechVisual, frontLeft, frontRight, backLeft, backRight, gyro, vision);
    }
}
