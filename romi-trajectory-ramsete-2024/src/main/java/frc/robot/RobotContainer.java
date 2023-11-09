// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionMultiCam;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.drive.pose.DifferentialPose;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain;
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channnel 0
  private final Joystick m_controller = new Joystick(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private VisionMultiCam vision;
  private static Alliance alliance;

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    vision = new VisionMultiCam("Romi", 0, AprilTags.aprilTagRoomLayout);
    m_drivetrain = new Drivetrain(vision);
    vision.setDrivetrainPoseEstimator(m_drivetrain.getPoseEstimator());

    vision.addPhotonCamera("PiCamera", 2, 
      new Transform3d(new Translation3d(0.087, 0, 0.07), new Rotation3d(0, 0, 180)), 
      PoseStrategy.LOWEST_AMBIGUITY, m_drivetrain.getPoseEstimator());
    //vision.setUpdateValues(true);
    // Configure the button bindings
    configureButtonBindings();
  }

  public static Alliance getAlliance() {
    return alliance;
  }
  /**
   * Generate a trajectory following Ramsete command
   * 
   * This is very similar to the WPILib RamseteCommand example. It uses
   * constants defined in the Constants.java file. These constants were 
   * found empirically by using the frc-characterization tool.
   * 
   * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
   */
  private Command generateRamseteCommand(Trajectory trajectory) {
    return generateRamseteCommand(trajectory, true, true);
  }

  private Command generateRamseteCommand(Trajectory trajectory, boolean reset, boolean stop) {
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        m_drivetrain::tankDriveVolts,
        m_drivetrain);

    m_drivetrain.resetOdometry(trajectory.getInitialPose());

    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(
        () -> { if (reset) m_drivetrain.resetOdometry(trajectory.getInitialPose()); }, m_drivetrain)
        
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> { if (stop) m_drivetrain.tankDriveVolts(0, 0); } , m_drivetrain));
  } 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    // Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    // onboardButtonA
    //     .whenActive(new PrintCommand("Button A Pressed"))
    //     .whenInactive(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Barrel Race", generateRamseteCommand(getTrajectoryFile(Constants.Trajectories.barrelRacePath)));
    m_chooser.addOption("Ramsete Trajectory", generateRamseteCommand(Constants.Trajectories.simpleTrajectory));
    m_chooser.addOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public static Trajectory getTrajectoryFile(String path) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    String trajectoryJSON = path;
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    return trajectory;
  }
  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> m_controller.getRawAxis(4), () -> -m_controller.getRawAxis(1));
  }
}
