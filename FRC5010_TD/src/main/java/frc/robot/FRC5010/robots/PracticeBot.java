// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionLimeLightSim;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.commands.JoystickToSwerve;
import frc.robot.FRC5010.commands.LogCommand;
import frc.robot.FRC5010.constants.AutoMaps;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.drive.swerve.MK4SwerveModule;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;
import frc.robot.FRC5010.subsystems.LedSubsystem;
import frc.robot.chargedup.DriverDisplaySubsystem;
import frc.robot.commands.AutoBalance;

/** Add your docs here. */
public class PracticeBot extends GenericMechanism {
  private GenericGyro gyro;
  private SwerveConstants swerveConstants;
  private DriverDisplaySubsystem driverDisplay;
  private AutoMaps autoMaps;
  private Drive drive;
  private LedSubsystem ledSubsystem;
  private SwerveDrivetrain swerveDrivetrain;

  public PracticeBot(Mechanism2d visual, ShuffleboardTab displayTab) {
    super(visual, displayTab);
    swerveConstants = new SwerveConstants(Units.inchesToMeters(24.25), Units.inchesToMeters(20.5));
    swerveConstants.setkFrontLeftAbsoluteOffsetRad(2.856);
    swerveConstants.setkFrontRightAbsoluteOffsetRad(2.444 + Math.PI);
    swerveConstants.setkBackLeftAbsoluteOffsetRad(2.858 + Math.PI);
    swerveConstants.setkBackRightAbsoluteOffsetRad(1.967);
    swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(4);
    swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);
    swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(3);
    swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);
    swerveConstants.setSwerveModuleConstants(MK4SwerveModule.MK4_L1);
    swerveConstants.configureSwerve(NEO.MAXRPM, NEO.MAXRPM);

    ShuffleboardTab visionTab = Shuffleboard.getTab("Drive");

    VisionSystem multiVision = new VisionLimeLightSim("", 0, AprilTags.aprilTagFieldLayout);

    visionTab.addCamera("DriverCam", "DriverCam", "10.50.10.11").withSize(3, 3);

    // VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1,
    // AprilTags.aprilTagFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP);
    // multiVision.addPhotonCamera("RightCamera",
    // new Transform3d( // This describes the vector between the camera lens to the
    // robot center on the
    // // ground
    // new Translation3d(Units.inchesToMeters(-2), Units.inchesToMeters(0.0),
    // Units.inchesToMeters(3.5)),
    // new Rotation3d(0, 0, Units.degreesToRadians(-100))));
    // multiVision.addPhotonCamera("LeftCamera",
    // new Transform3d( // This describes the vector between the camera lens to the
    // robot center on the
    // // ground
    // new Translation3d(Units.inchesToMeters(-5.5), 0, Units.inchesToMeters(3.5)),
    // new Rotation3d(0, 0, Units.degreesToRadians(80))));

    List<SwervePorts> swervePorts = new ArrayList<>();
    swervePorts.add(new SwervePorts(1, 3, 21));
    swervePorts.add(new SwervePorts(10, 8, 22));
    swervePorts.add(new SwervePorts(5, 4, 23));
    swervePorts.add(new SwervePorts(2, 7, 20));

    gyro = new PigeonGyro(11);

    drive = new Drive(multiVision, gyro, Drive.Type.YAGSL_MK4_SWERVE_DRIVE, swervePorts, swerveConstants);
    ledSubsystem = new LedSubsystem(1, 60);
    // multiVision.setDrivetrainPoseEstimator(drive.getDrivetrain().getPoseEstimator());

    driverDisplay = new DriverDisplaySubsystem(drive.getDrivetrain().getPoseEstimator());

    autoMaps = new AutoMaps();
    swerveDrivetrain = (SwerveDrivetrain) drive.getDrivetrain();

    // Drivetrain Controls
    autoMaps.addMarker("AutoBalance", new AutoBalance(swerveDrivetrain, () -> false, gyro));
    autoMaps.addMarker("ExtendToPivotPosition", new LogCommand("ExtendToPivotPosition"));
    autoMaps.addMarker("HomeElevator", new LogCommand("HomeElevator"));
    autoMaps.addMarker("PivotToGround", new LogCommand("PivotToGround"));
    autoMaps.addMarker("PivotToLow", new LogCommand("PivotToLow"));
    autoMaps.addMarker("PivotToMid", new LogCommand("PivotToMid"));
    autoMaps.addMarker("LockWheels", new InstantCommand(() -> swerveDrivetrain.lockWheels()));
    autoMaps.addMarker("Yeet Cube", new LogCommand("Cube has been yeeted"));
    // autoMaps.addMarker("PivotToHigh", new PivotElevator(pivotSubsystem,
    // ElevatorLevel.high));
    autoMaps.addMarker("HomePivot", new LogCommand("HomePivot"));

    // Intake Controls
    autoMaps.addMarker("ConeMode", new LogCommand("ConeMode"));
    autoMaps.addMarker("CubeMode", new LogCommand("CubeMode"));
    autoMaps.addMarker("Outtake", new LogCommand("Outtake"));
    autoMaps.addMarker("Intake", new LogCommand("Intake"));

    // Drivetrain Controls
    autoMaps.addMarker("AutoExtendDrop", new LogCommand("AutoExtendDrop"));
    // Create
    // autoMaps.addPath("8-1 Cube", new PathConstraints(4, 3));
    // autoMaps.addPath("7-2 North Cone", new PathConstraints(1, 0.5));
    autoMaps.addPath("6-3 Three Cubes", new PathConstraints(2, 1));
    autoMaps.addPath("Bal Direct 7-2 Cube", new PathConstraints(1.5, .75));
    autoMaps.addPath("Bal Over 7-2 Cube", new PathConstraints(1.5, .75));
    autoMaps.addPath("Bal Over 7-2 Slow Cube", new PathConstraints(1.75, 1));
    autoMaps.addPath("8-1 Three Cubes", new PathConstraints(4, 1.75));

  }

  @Override
  public Map<String, List<PathPlannerTrajectory>> initAutoCommands() {
    return drive.setAutoCommands(autoMaps.getPaths(), autoMaps.getEventMap());
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.createYButton()
        .whileTrue(new AutoBalance(drive.getDrivetrain(), () -> !driver.createStartButton().getAsBoolean(), gyro));
    driver.createAButton()
        .whileTrue(new JoystickToSwerve(swerveDrivetrain, () -> 0.5, () -> 0.0, () -> 0.5, () -> true));

    // driver.createBButton().whileTrue(new DriveToPosition((SwerveDrivetrain)
    // drive.getDrivetrain(),
    // () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
    // () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
    // ledSubsystem, LCR.left));

    // driver.createAButton().whileTrue(new DriveToPosition((SwerveDrivetrain)
    // drive.getDrivetrain(),
    // () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
    // () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
    // ledSubsystem, LCR.center));

    // driver.createXButton().whileTrue(new DriveToPosition((SwerveDrivetrain)
    // drive.getDrivetrain(),
    // () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
    // () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(),
    // ledSubsystem, LCR.right));

    SwerveDrivetrain swerveDrivetrain = (SwerveDrivetrain) drive.getDrivetrain();
    driver.createLeftBumper().whileTrue(new RunCommand(() -> swerveDrivetrain.lockWheels(), swerveDrivetrain));

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
  public Command generateAutoCommand(List<PathPlannerTrajectory> paths) {
    return drive.generateAutoCommand(paths);
  }
}
