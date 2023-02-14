// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.VisionLimeLightSim;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.commands.AutoModes;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.drive.swerve.MK4iSwerveModule;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.mechanisms.DriveConstantsDef;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;

/** Add your docs here. */
public class CompBot extends GenericMechanism {
    private SwerveConstants swerveConstants; 
    private Drive drive; 
    private GenericMechanism elevator;
    private AutoModes autoMaps;

    private Persisted<Double> maxChassisVelocity;
    private Persisted<Double> maxChassisRotation;
    public CompBot(Mechanism2d visual, ShuffleboardTab displayTab) {
      super(visual, displayTab);
        // Needs to be set
        swerveConstants = new SwerveConstants(Units.inchesToMeters(22), Units.inchesToMeters(26.5));

        // Baby Swerve values need to be changed
        swerveConstants.setkFrontLeftAbsoluteOffsetRad(-1.6383);
        swerveConstants.setkFrontRightAbsoluteOffsetRad(-.85596);
        swerveConstants.setkBackLeftAbsoluteOffsetRad(.1656699);
        swerveConstants.setkBackRightAbsoluteOffsetRad(-.002);

        swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(5);
        swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);

        swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(.4);
        swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);

        swerveConstants.setSwerveModuleConstants(MK4iSwerveModule.MK4I_L1);
        swerveConstants.configureSwerve(NEO.MAXRPM, NEO.MAXRPM);
        maxChassisVelocity = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_VELOCITY, swerveConstants.getkTeleDriveMaxSpeedMetersPerSecond());
        maxChassisRotation = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_ROTATION, swerveConstants.getkTeleDriveMaxAngularSpeedRadiansPerSecond());
        
        // Will need to be changed for 2023 field
        VisionSystem multiVision = new VisionLimeLightSim("Sim", 0);
        // VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1, AprilTags.aprilTagRoomLayout, PoseStrategy.AVERAGE_BEST_TARGETS);
        // multiVision.addPhotonCamera("Arducam_OV9281_USB_Camera", 
        //   new Transform3d( // This describes the vector between the camera lens to the robot center on the ground
        //     new Translation3d(Units.inchesToMeters(7), 0, Units.inchesToMeters(16.75)), 
        //     new Rotation3d(0, Units.degreesToRadians(-20), 0)
        //   )
        // );

        // Ports need to be changed when comp bot is ready
        List<SwervePorts> swervePorts = new ArrayList<>();
        swervePorts.add(new SwervePorts(1, 2, 10)); //FL
        swervePorts.add(new SwervePorts(8, 7, 11)); //FR
        swervePorts.add(new SwervePorts(3, 4, 12)); //BL
        swervePorts.add(new SwervePorts(5, 6, 9));  //BR


        GenericGyro gyro = new PigeonGyro(13);

        autoMaps = new ChargedUpAutoModes();
        autoMaps.loadAutoPaths();

        drive = new Drive(multiVision, gyro, Drive.Type.SDS_MK4I_SWERVE_DRIVE, swervePorts, swerveConstants);
        // Uncomment when using PhotonVision
        // multiVision.setDrivetrainPoseEstimator(drive.getDrivetrain().getPoseEstimator());
        elevator = new ChargedUpMech(mechVisual, shuffleTab);
    } 

    public Map<String,Command> setAutoCommands(){
      return drive.setAutoCommands(autoMaps.getPaths(), autoMaps.getEventMap()); 
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
      drive.configureButtonBindings(driver, operator);      
      //elevator.configureButtonBindings(driver, operator);
    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
      drive.setupDefaultCommands(driver, operator);
      //elevator.setupDefaultCommands(driver, operator);
    }

    @Override
    protected void initRealOrSim() {
    }

    @Override
    public Map<String, Command> initAutoCommands() {
      return drive.setAutoCommands(autoMaps.getPaths(), autoMaps.getEventMap());
    }
}
