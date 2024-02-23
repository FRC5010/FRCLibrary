// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionMultiCam;
import frc.robot.FRC5010.arch.GenericMechanism;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.drive.swerve.ThriftySwerveModule;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.motors.hardware.NEO550;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.NavXGyro;
import frc.robot.FRC5010.subsystems.DriverDisplaySubsystem;

/**
 * To setup a copy of this class
 * Copy to a new class and rename the constructor
 * In RobotFactory add constants and a switch-case for your class
 * Start PhotonVision and get the camera name (java -jar photonvision.jar)
 * Start the robot simulation
 * In Glass - find the NetworkTables->Preferences->WhoAmI and set it to your
 * robot name
 * Also change the LaptopCamera name from photon vision
 * Stop the simulator, restart and it should start your robot code
 */
public class CurtsLaptopSimulator extends GenericMechanism {
    Drive drive;
    SwerveConstants swerveConstants;
    private DriverDisplaySubsystem driverDiplay;

    public CurtsLaptopSimulator(Mechanism2d visual, ShuffleboardTab displayTab) {
        super(visual, displayTab);
        swerveConstants = new SwerveConstants(0.76835, 0.635);
        swerveConstants.setkFrontLeftAbsoluteOffsetRad(0.26);
        swerveConstants.setkFrontRightAbsoluteOffsetRad(-3.14);
        swerveConstants.setkBackLeftAbsoluteOffsetRad(1.0 + Math.PI);
        swerveConstants.setkBackRightAbsoluteOffsetRad(0.21 + Math.PI);
        swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(5);
        swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);
        swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(.4);
        swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
        swerveConstants.setSwerveModuleConstants(ThriftySwerveModule.moduleConstants);
        swerveConstants.configureSwerve(NEO.MAXRPM, NEO550.MAXRPM);

        VisionMultiCam multiVision = new VisionMultiCam("Vision", 0,
                AprilTags.aprilTagFieldLayout);
        List<SwervePorts> swervePorts = new ArrayList<>();
        swervePorts.add(new SwervePorts(1, 2, 0));
        swervePorts.add(new SwervePorts(7, 8, 1));
        swervePorts.add(new SwervePorts(3, 4, 2));
        swervePorts.add(new SwervePorts(5, 6, 3));

        GenericGyro gyro = new NavXGyro(SPI.Port.kMXP);

        drive = new Drive(multiVision, gyro, Drive.Type.YAGSL_SWERVE_DRIVE, swervePorts, swerveConstants,
                "swervemk4i");
        // multiVision.addPhotonCamera(Persisted.stringVal(VisionConstantDefs.LAPTOP_CAMERA),
        // 1,
        // new Transform3d(
        // // This describes the vector between the camera lens to the robot center on
        // the
        // // ground
        // new Translation3d(Units.inchesToMeters(7), 0, Units.inchesToMeters(16.75)),
        // new Rotation3d(0, Units.degreesToRadians(-20), 0)),
        // PoseStrategy.AVERAGE_BEST_TARGETS, drive.getDrivetrain().getPoseEstimator());
        driverDiplay = new DriverDisplaySubsystem(drive.getDrivetrain().getPoseEstimator());
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        drive.configureButtonBindings(driver, operator);


    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        drive.setupDefaultCommands(driver, operator);
    }

    @Override
    protected void initRealOrSim() {
    }

    @Override
    public void initAutoCommands() {
        drive.initAutoCommands();
    }

    @Override
    public Command generateAutoCommand(Command autoCommand) {
      return drive.generateAutoCommand(autoCommand);
    }
}
