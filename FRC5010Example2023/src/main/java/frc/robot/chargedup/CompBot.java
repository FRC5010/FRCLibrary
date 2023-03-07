// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionLimeLightSim;
import frc.robot.FRC5010.Vision.VisionPhotonMultiCam;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.commands.ElapseTime;
import frc.robot.FRC5010.constants.AutoMaps;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.constants.SwervePorts;
import frc.robot.FRC5010.drive.swerve.MK4iSwerveModule;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.ButtonBoard;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;
import frc.robot.FRC5010.subsystems.LedSubsystem;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.HomePivot;
import frc.robot.commands.DriveToPosition.LCR;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.PivotElevator;

/** Add your docs here. */
public class CompBot extends GenericMechanism {
  private SwerveConstants swerveConstants;
  private Drive drive;
  private DriverDisplaySubsystem driverDiplay;
  private GenericMechanism elevator;
  private AutoMaps autoMaps;
  private ButtonBoard buttonOperator;
  private LedSubsystem ledSubsystem;
  private GenericGyro gyro;

  public CompBot(Mechanism2d visual, ShuffleboardTab displayTab) {
    super(visual, displayTab);
    // Needs to be set
    swerveConstants = new SwerveConstants(Units.inchesToMeters(22), Units.inchesToMeters(26.5));

    // Baby Swerve values need to be changed
    swerveConstants.setkFrontLeftAbsoluteOffsetRad(Units.degreesToRadians(-93.076)); //
    swerveConstants.setkFrontRightAbsoluteOffsetRad(Units.degreesToRadians(140.010)); //
    swerveConstants.setkBackLeftAbsoluteOffsetRad(Units.degreesToRadians(-1.582)); //
    swerveConstants.setkBackRightAbsoluteOffsetRad(Units.degreesToRadians(1.670)); //
    // swerveConstants.setkFrontLeftAbsoluteOffsetRad(0); //
    // swerveConstants.setkFrontRightAbsoluteOffsetRad(0); //
    // swerveConstants.setkBackLeftAbsoluteOffsetRad(0); //
    // swerveConstants.setkBackRightAbsoluteOffsetRad(0);

    swerveConstants.setkTeleDriveMaxSpeedMetersPerSecond(5);
    swerveConstants.setkTeleDriveMaxAngularSpeedRadiansPerSecond(6);

    swerveConstants.setkTeleDriveMaxAccelerationUnitsPerSecond(.1);
    swerveConstants.setkTeleDriveMaxAngularAccelerationUnitsPerSecond(5 * Math.PI);

    swerveConstants.setSwerveModuleConstants(MK4iSwerveModule.MK4I_L1);
    swerveConstants.configureSwerve(NEO.MAXRPM, NEO.MAXRPM);

    ledSubsystem = new LedSubsystem(1, 290);
    ledSubsystem.off();

    // Will need to be changed for 2023 field
    VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1, AprilTags.aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP);

    // y = +- 27.75 / 2, x = 2.5, z = 36.75
    multiVision.addPhotonCamera("LeftCamera",
        new Transform3d( // This describes the vector between the camera lens to the robot center on the
                         // ground
            new Translation3d(Units.inchesToMeters(27.75 / 2), Units.inchesToMeters(2.5), Units.inchesToMeters(36.75)),
            new Rotation3d(0, 0, Units.degreesToRadians(90))));

    multiVision.addPhotonCamera("RightCamera",
        new Transform3d( // This describes the vector between the camera lens to the robot center on the
                         // ground
            new Translation3d(-Units.inchesToMeters(27.75 / 2), Units.inchesToMeters(2.5), Units.inchesToMeters(36.75)),
            new Rotation3d(0, 0, Units.degreesToRadians(-90))));

    // Ports need to be changed when comp bot is ready
    List<SwervePorts> swervePorts = new ArrayList<>();

    swervePorts.add(new SwervePorts(1, 2, 10)); // FL
    swervePorts.add(new SwervePorts(8, 7, 11)); // FR
    swervePorts.add(new SwervePorts(3, 4, 12)); // BL
    swervePorts.add(new SwervePorts(5, 6, 9)); // BR

    gyro = new PigeonGyro(13);

    drive = new Drive(multiVision, gyro, Drive.Type.YAGSL_MK4I_SWERVE_DRIVE, swervePorts, swerveConstants);
    // Uncomment when using PhotonVision

    multiVision.setDrivetrainPoseEstimator(drive.getDrivetrain().getPoseEstimator());
    driverDiplay = new DriverDisplaySubsystem(drive.getDrivetrain().getPoseEstimator());

    buttonOperator = new ButtonBoard(Controller.JoystickPorts.TWO.ordinal());
    buttonOperator.createButtons(11);
    elevator = new ChargedUpMech(mechVisual, shuffleTab, buttonOperator, ledSubsystem);

    autoMaps = new AutoMaps();
    SwerveDrivetrain swerveDrivetrain = (SwerveDrivetrain) drive.getDrivetrain();
    ElevatorSubsystem elevatorSubsystem = ((ChargedUpMech) elevator).getElevatorSubsystem();
    IntakeSubsystem intakeSubsystem = ((ChargedUpMech) elevator).getIntakeSubsystem();
    PivotSubsystem pivotSubsystem = ((ChargedUpMech) elevator).getPivotSubsystem();

    // Elevator Controls
     autoMaps.addMarker("ExtendToPivotPosition", new
     MoveElevator(elevatorSubsystem, () -> ElevatorLevel.low));
    autoMaps.addMarker("HomeElevator", new HomeElevator(elevatorSubsystem));
    autoMaps.addMarker("PivotToGround", new PivotElevator(pivotSubsystem,
    ElevatorLevel.ground));
    autoMaps.addMarker("PivotToLow", new PivotElevator(pivotSubsystem,
    ElevatorLevel.low));
    autoMaps.addMarker("PivotToMid", new PivotElevator(pivotSubsystem,
    ElevatorLevel.medium));
    // autoMaps.addMarker("PivotToHigh", new PivotElevator(pivotSubsystem,
    // ElevatorLevel.high));
    autoMaps.addMarker("HomePivot", new HomePivot(pivotSubsystem));

    // Intake Controls
    autoMaps.addMarker("ConeMode", new InstantCommand(() -> intakeSubsystem.setIntakeCone(), intakeSubsystem));
    autoMaps.addMarker("CubeMode", new InstantCommand(() -> intakeSubsystem.setIntakeCube(), intakeSubsystem));
    autoMaps.addMarker("Outtake",(new IntakeSpin(intakeSubsystem, () -> -0.6).withTimeout(.5)));
    autoMaps.addMarker("Intake",(new IntakeSpin(intakeSubsystem, () -> 0.6).withTimeout(0.5)));
    // Drivetrain Controls
    autoMaps.addMarker("AutoBalance", new AutoBalance(swerveDrivetrain, () -> false, gyro));

    autoMaps.addMarker("AutoExtendDrop", new MoveElevator(elevatorSubsystem, () -> ElevatorLevel.high)
    .andThen(new IntakeSpin(intakeSubsystem, () -> 0.2))
    .withTimeout(0.5)
    .andThen(new HomeElevator(elevatorSubsystem))
    .andThen(new HomePivot(pivotSubsystem)));
    // Create Paths
    autoMaps.addPath("8-1 North Cone", new PathConstraints(4, 3));
    autoMaps.addPath("7-2 North Cone", new PathConstraints(1, 0.5));
    autoMaps.addPath("6-3 Cube", new PathConstraints(4, 3));
    autoMaps.addPath("Bal Direct 7-2 Cube", new PathConstraints(1.5, .75));
    autoMaps.addPath("Bal Over 7-2 Cube", new PathConstraints(1.5, .75));
  }

  public Map<String, Command> setAutoCommands() {
    return drive.setAutoCommands(autoMaps.getPaths(), autoMaps.getEventMap());
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.createYButton()
        .whileTrue(new AutoBalance(drive.getDrivetrain(), () -> !driver.createLeftBumper().getAsBoolean(), gyro));

    driver.createBButton().whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
        () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
        () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(), ledSubsystem, LCR.left));

    driver.createAButton().whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
        () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
        () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(), ledSubsystem, LCR.center));

    driver.createXButton().whileTrue(new DriveToPosition((SwerveDrivetrain) drive.getDrivetrain(),
        () -> drive.getDrivetrain().getPoseEstimator().getCurrentPose(),
        () -> drive.getDrivetrain().getPoseEstimator().getPoseFromClosestTag(), ledSubsystem, LCR.right));

    drive.configureButtonBindings(driver, operator);
    elevator.configureButtonBindings(driver, operator);
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    drive.setupDefaultCommands(driver, operator);
    elevator.setupDefaultCommands(driver, operator);
  }

  @Override
  protected void initRealOrSim() {
  }

  @Override
  public Map<String, Command> initAutoCommands() {
    return drive.setAutoCommands(autoMaps.getPaths(), autoMaps.getEventMap());
  }

  public void disabledBehavior() {
    drive.disabledBehavior();
  }
}
