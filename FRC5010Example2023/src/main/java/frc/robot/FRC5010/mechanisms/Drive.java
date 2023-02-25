// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.mechanisms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.commands.DefaultDriveCommand;
import frc.robot.FRC5010.constants.DrivePorts;
import frc.robot.FRC5010.constants.GenericDrivetrainConstants;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.constants.RobotConstantsDef;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwerveModuleConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.drive.DifferentialDrivetrain;
import frc.robot.FRC5010.drive.GenericDrivetrain;
import frc.robot.FRC5010.drive.swerve.GenericSwerveModule;
import frc.robot.FRC5010.drive.swerve.MK4SwerveModule;
import frc.robot.FRC5010.drive.swerve.MK4iSwerveModule;
import frc.robot.FRC5010.drive.swerve.SdsSwerveModule;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.drive.swerve.ThriftySwerveModule;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.subsystems.LedSubsystem;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.DriveToPosition.LCR;

/** Add your docs here. */
public class Drive extends GenericMechanism {
    private VisionSystem vision;
    private GenericDrivetrain drivetrain;
    private GenericGyro gyro;

    private Command defaultDriveCommand;
    private String type;
    private GenericDrivetrainConstants driveConstants;
    private List<? extends DrivePorts> motorPorts;
    private Persisted<Double> maxChassisVelocity;
    private Persisted<Double> maxChassisRotation;

    public static class Type {
        public static final String DIFF_DRIVE = "DifferentialDrive";
        public static final String THRIFTY_SWERVE_DRIVE = "ThriftySwerveDrive";
        public static final String MK4_SWERVE_DRIVE = "MK4SwerveDrive";
        public static final String MK4I_SWERVE_DRIVE = "MK4ISwerveDrive";
        public static final String SDS_MK4I_SWERVE_DRIVE = "SDSMK4ISwerveDrive";
        public static final String SDS_MK4_SWERVE_DRIVE = "SDSMK4SwerveDrive";
    }

    // Examples of how to use a persisted constants
    // These can live in specific constants files, however
    private static Persisted<Integer> driveVisualH;
    private static Persisted<Integer> driveVisualV;

    public Drive(VisionSystem visionSystem, GenericGyro gyro, String type, List<? extends DrivePorts> drivePorts,
            GenericDrivetrainConstants driveConstants) {
        super(Drive.class.getSimpleName());
        this.vision = visionSystem;
        this.gyro = gyro;
        this.type = type;
        this.motorPorts = drivePorts;
        this.driveConstants = driveConstants;
        driveVisualH = new Persisted<>(RobotConstantsDef.DRIVE_VISUAL_H, 60);
        driveVisualV = new Persisted<>(RobotConstantsDef.DRIVE_VISUAL_V, 60);
        maxChassisVelocity = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_VELOCITY,
                driveConstants.getkTeleDriveMaxSpeedMetersPerSecond());
        maxChassisRotation = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_ROTATION,
                driveConstants.getkTeleDriveMaxAngularSpeedRadiansPerSecond());
        mechVisual = new Mechanism2d(driveVisualH.getInteger(), driveVisualV.getInteger());
        SmartDashboard.putData("Drive Visual", mechVisual);
        initRealOrSim();
    }

    @Override
    protected void initRealOrSim() {
        switch (type) {
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
                initializeMK4iSwerveDrive();
                break;
            }
            case Type.SDS_MK4I_SWERVE_DRIVE: {
                initializeSDSMk4iSwerveDrive();
                break;
            }
            case Type.SDS_MK4_SWERVE_DRIVE:{
                initializeSDSMk4SwerveDrive();
                break;
            }
            default: {
                break;
            }
        }
    }

    public void setupDefaultCommands(Controller driver, Controller operator) {
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

        driver.setLeftXAxis(driver.createLeftXAxis().negate().deadzone(0.08).cubed());
        driver.setLeftYAxis(driver.createLeftYAxis().negate().deadzone(0.08).cubed());
        driver.setRightXAxis(driver.createRightXAxis().negate().deadzone(0.08));
        // driver.createXButton().whileTrue(new ChaseTag((SwerveDrivetrain) drivetrain,
        // () -> drivetrain.getPoseEstimator().getCurrentPose()));

        // Example of setting up axis for driving omnidirectional
        // driver.setLeftXAxis(driver.createLeftXAxis()
        // .negate().deadzone(0.07).limit(1).rate(2).cubed());
        // driver.setLeftYAxis(driver.createLeftYAxis()
        // .negate().deadzone(0.07).limit(1).rate(2).cubed());
        // driver.setRightXAxis(driver.createRightXAxis()
        // .negate().deadzone(0.07).limit(1).rate(4).cubed());
        // Put commands that can be both real and simulation afterwards

        defaultDriveCommand = new DefaultDriveCommand(drivetrain,
                () -> driver.getLeftYAxis(),
                () -> driver.getLeftXAxis(),
                () -> driver.getRightXAxis(),
                () -> !driver.createLeftBumper().getAsBoolean());

        driver.createStartButton().onTrue(new InstantCommand(() -> gyro.reset()));
        
    }

    public GenericDrivetrain getDrivetrain() {
        return drivetrain;
    }

    private void initializeThriftySwerveDrive() {
        SwerveModuleConstants frontLeftConstants = new SwerveModuleConstants(0, 0, false, 0, true, true);
        SwerveModuleConstants frontRightConstants = new SwerveModuleConstants(0, 0, false, 0, true, true);
        SwerveModuleConstants backLeftConstants = new SwerveModuleConstants(0, 0, true, 0, true, true);
        SwerveModuleConstants backRightConstants = new SwerveModuleConstants(0, 0, true, 0, true, true);

        GenericSwerveModule frontLeft = new ThriftySwerveModule(
                mechVisual.getRoot("frontleft", 15, 45), "frontleft",
                ((SwerveConstants) driveConstants).getkFrontLeftAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(0),
                frontLeftConstants, (SwerveConstants) driveConstants);
        GenericSwerveModule frontRight = new ThriftySwerveModule(
                mechVisual.getRoot("frontright", 45, 45), "frontright",
                ((SwerveConstants) driveConstants).getkFrontRightAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(1),
                frontRightConstants, (SwerveConstants) driveConstants);
        GenericSwerveModule backLeft = new ThriftySwerveModule(
                mechVisual.getRoot("backleft", 15, 15), "backleft",
                ((SwerveConstants) driveConstants).getkBackLeftAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(2),
                backLeftConstants, (SwerveConstants) driveConstants);
        GenericSwerveModule backRight = new ThriftySwerveModule(
                mechVisual.getRoot("backright", 45, 15), "backright",
                ((SwerveConstants) driveConstants).getkFrontLeftAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(3),
                backRightConstants, (SwerveConstants) driveConstants);

        drivetrain = new SwerveDrivetrain(mechVisual, frontLeft, frontRight, backLeft, backRight, gyro, vision,
                (SwerveConstants) driveConstants);
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
        SwerveModuleConstants frontLeftConstants = new SwerveModuleConstants(0, 0, true, 0, false, false);
        SwerveModuleConstants frontRightConstants = new SwerveModuleConstants(0, 0, true, 0, false, false);
        SwerveModuleConstants backLeftConstants = new SwerveModuleConstants(0, 0, true, 0, false, false);
        SwerveModuleConstants backRightConstants = new SwerveModuleConstants(0, 0, true, 0, false, false);

        GenericSwerveModule frontLeft = new MK4SwerveModule(
                mechVisual.getRoot("frontleft", 15, 45), "frontleft",
                ((SwerveConstants) driveConstants).getkFrontLeftAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(0),
                frontLeftConstants, (SwerveConstants) driveConstants);
        GenericSwerveModule frontRight = new MK4SwerveModule(
                mechVisual.getRoot("frontright", 45, 45), "frontright",
                ((SwerveConstants) driveConstants).getkFrontRightAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(1),
                frontRightConstants, (SwerveConstants) driveConstants);
        GenericSwerveModule backLeft = new MK4SwerveModule(
                mechVisual.getRoot("backleft", 15, 15), "backleft",
                ((SwerveConstants) driveConstants).getkBackLeftAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(2),
                backLeftConstants, (SwerveConstants) driveConstants);
        GenericSwerveModule backRight = new MK4SwerveModule(
                mechVisual.getRoot("backright", 45, 15), "backright",
                ((SwerveConstants) driveConstants).getkBackRightAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(3),
                backRightConstants, (SwerveConstants) driveConstants);

        drivetrain = new SwerveDrivetrain(mechVisual, frontLeft, frontRight, backLeft, backRight, gyro, vision,
                (SwerveConstants) driveConstants);
    }

    private void initializeMK4iSwerveDrive() {
        SwerveModuleConstants frontLeftConstants = new SwerveModuleConstants(0, 0, false, 0, true, false);
        SwerveModuleConstants frontRightConstants = new SwerveModuleConstants(0, 0, false, 0, true, false);
        SwerveModuleConstants backLeftConstants = new SwerveModuleConstants(0, 0, false, 0, true, false);
        SwerveModuleConstants backRightConstants = new SwerveModuleConstants(0, 0, false, 0, true, false);

        GenericSwerveModule frontLeft = new MK4iSwerveModule(
                mechVisual.getRoot("frontleft", 15, 45), "frontleft",
                ((SwerveConstants) driveConstants).getkFrontLeftAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(0),
                frontLeftConstants, (SwerveConstants) driveConstants);
        GenericSwerveModule frontRight = new MK4iSwerveModule(
                mechVisual.getRoot("frontright", 45, 45), "frontright",
                ((SwerveConstants) driveConstants).getkFrontRightAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(1),
                frontRightConstants, (SwerveConstants) driveConstants);
        GenericSwerveModule backLeft = new MK4iSwerveModule(
                mechVisual.getRoot("backleft", 15, 15), "backleft",
                ((SwerveConstants) driveConstants).getkBackLeftAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(2),
                backLeftConstants, (SwerveConstants) driveConstants);
        GenericSwerveModule backRight = new MK4iSwerveModule(
                mechVisual.getRoot("backright", 45, 15), "backright",
                ((SwerveConstants) driveConstants).getkBackRightAbsoluteOffsetRad(), (SwervePorts) motorPorts.get(3),
                backRightConstants, (SwerveConstants) driveConstants);

        drivetrain = new SwerveDrivetrain(mechVisual, frontLeft, frontRight, backLeft, backRight, gyro, vision,
                (SwerveConstants) driveConstants);
    }

    private void initializeSDSMk4iSwerveDrive() {
        SdsSwerveModule frontLeft = new SdsSwerveModule(mechVisual.getRoot("frontLeft", 15, 45), "frontLeft", 
        ((SwerveConstants) driveConstants).getkFrontLeftAbsoluteOffsetRad(),
        ((SwerveConstants) driveConstants), (SwervePorts)motorPorts.get(0), SdsModuleConfigurations.MK4I_L1);

        SdsSwerveModule frontRight = new SdsSwerveModule(mechVisual.getRoot("frontRight", 45, 45), "frontRight", 
        ((SwerveConstants) driveConstants).getkFrontRightAbsoluteOffsetRad(),
        ((SwerveConstants) driveConstants), (SwervePorts)motorPorts.get(1), SdsModuleConfigurations.MK4I_L1);

        SdsSwerveModule backLeft = new SdsSwerveModule(mechVisual.getRoot("backLeft", 15,15), "backLeft", 
        ((SwerveConstants) driveConstants).getkBackLeftAbsoluteOffsetRad(),
        ((SwerveConstants) driveConstants), (SwervePorts)motorPorts.get(2), SdsModuleConfigurations.MK4I_L1);

        SdsSwerveModule backRight = new SdsSwerveModule(mechVisual.getRoot("backRight", 45,15), "backRight", 
        ((SwerveConstants) driveConstants).getkBackRightAbsoluteOffsetRad(),
        ((SwerveConstants) driveConstants), (SwervePorts)motorPorts.get(3), SdsModuleConfigurations.MK4I_L1);

        drivetrain = new SwerveDrivetrain(mechVisual, frontLeft, frontRight, backLeft, backRight, gyro, vision,
                (SwerveConstants) driveConstants);
//        drivetrain = new SdsSwerveDrivetrain(mechVisual, frontLeft, frontRight, backLeft, backRight, gyro, (SwerveConstants)driveConstants, vision);
    }

    private void initializeSDSMk4SwerveDrive() {
        SdsSwerveModule frontLeft = new SdsSwerveModule(mechVisual.getRoot("frontLeft", 15, 45), "frontLeft", 
        ((SwerveConstants) driveConstants).getkFrontLeftAbsoluteOffsetRad(),
        ((SwerveConstants) driveConstants), (SwervePorts)motorPorts.get(0), SdsModuleConfigurations.MK4_L1);

        SdsSwerveModule frontRight = new SdsSwerveModule(mechVisual.getRoot("frontRight", 45, 45), "frontRight", 
        ((SwerveConstants) driveConstants).getkFrontRightAbsoluteOffsetRad(),
        ((SwerveConstants) driveConstants), (SwervePorts)motorPorts.get(1), SdsModuleConfigurations.MK4_L1);

        SdsSwerveModule backLeft = new SdsSwerveModule(mechVisual.getRoot("backLeft", 15,15), "backLeft", 
        ((SwerveConstants) driveConstants).getkBackLeftAbsoluteOffsetRad(),
        ((SwerveConstants) driveConstants), (SwervePorts)motorPorts.get(2), SdsModuleConfigurations.MK4_L1);

        SdsSwerveModule backRight = new SdsSwerveModule(mechVisual.getRoot("backRight", 15,15), "backRight", 
        ((SwerveConstants) driveConstants).getkBackRightAbsoluteOffsetRad(),
        ((SwerveConstants) driveConstants), (SwervePorts)motorPorts.get(3), SdsModuleConfigurations.MK4_L1);

        drivetrain = new SwerveDrivetrain(mechVisual, frontLeft, frontRight, backLeft, backRight, gyro, vision, (SwerveConstants)driveConstants);
    }

    public Map<String, Command> initAutoCommands() {
        return new HashMap<>();
    }

    public Map<String, Command> setAutoCommands(Map<String, List<PathPlannerTrajectory>> paths,
            HashMap<String, Command> eventMap) {
        Map<String, Command> commands = new HashMap<>();
        BaseAutoBuilder autoBuilder = drivetrain.setAutoBuilder(eventMap);

        for (String name : paths.keySet()) {
            List<PathPlannerTrajectory> path = paths.get(name);
            commands.put(name, autoBuilder.fullAuto(path));
        }

        return commands;
    }
}