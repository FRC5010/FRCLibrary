// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.swerve;

import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.commands.JoystickToSwerve;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.pose.YAGSLSwervePose;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Add your docs here. */
public class YAGSLSwerveDrivetrain extends SwerveDrivetrain {
  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;

  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  public double maximumSpeed = Units.feetToMeters(14.5);

  public YAGSLSwerveDrivetrain(Mechanism2d mechVisual, GenericGyro gyro, SwerveConstants swerveConstants,
      String swerveType, VisionSystem visionSystem) {
    super(mechVisual, gyro, swerveConstants);

    /** 5010 Code */
    this.maximumSpeed = swerveConstants.getkPhysicalMaxSpeedMetersPerSecond();
    /** END 5010 Code */

    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(21.42, 1);
    // Motor conversion factor is (PI * WHEEL DIAMETER) / (GEAR RATIO * ENCODER
    // RESOLUTION).
    // In this case the wheel diameter is 4 inches.
    // The gear ratio is 6.75 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.12, 1);
    System.out.println("\"conversionFactor\": {");
    System.out.println("\t\"angle\": " + angleConversionFactor + ",");
    System.out.println("\t\"drive\": " + driveConversionFactor);
    System.out.println("}");

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), swerveType);
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      System.out.println(e.getMessage());
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via
                                             // angle.

    /** 5010 Code */
    poseEstimator = new DrivetrainPoseEstimator(new YAGSLSwervePose(gyro, this), visionSystem);
    setDrivetrainPoseEstimator(poseEstimator);

    Shuffleboard.getTab("Drive").addBoolean("Has Issues", () -> hasIssues()).withPosition(9, 1);
    if (RobotBase.isSimulation() || useGlass) {
      initGlassWidget();
    }
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    poseEstimator.update();
    hasIssues();
    if (RobotBase.isSimulation() || useGlass) {
      updateGlassWidget();
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    /** 5010 Code */
    swerveDrive.setGyro(new Rotation3d(0, 0, initialHolonomicPose.getRotation().getRadians()));
    /** END 5010 Code */
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public void updateOdometry() {
    swerveDrive.updateOdometry();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW
   * positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        angle.getRadians(),
        getHeading().getRadians(),
        maximumSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  @Override
  public void lockWheels() {
    swerveDrive.lockPose();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /** 5010 Code */

  public Command createDefaultCommand(Controller driverXbox) {
    // System.out.println("brrrrr");
    DoubleSupplier leftX = () -> driverXbox.getLeftXAxis();
    DoubleSupplier leftY = () -> driverXbox.getLeftYAxis();
    DoubleSupplier rightX = () -> driverXbox.getRightXAxis();
    BooleanSupplier isFieldOriented = () -> isFieldOrientedDrive;

    return new JoystickToSwerve(this, leftY, leftX, rightX, isFieldOriented);
    // return new TeleopDrive(this, leftX, leftY, rightX, isFieldOriented);
  }

  @Override
  public void stop() {
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public void resetEncoders() {
    swerveDrive.resetDriveEncoders();
    swerveDrive.synchronizeModuleEncoders();
  }

  @Override
  public void drive(ChassisSpeeds direction) {
    // Thank you to Jared Russell FRC254 for Open Loop Compensation Code
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
    double dtConstant = 0.009;
    Pose2d robotPoseVel = new Pose2d(direction.vxMetersPerSecond * dtConstant,
        direction.vyMetersPerSecond * dtConstant,
        Rotation2d.fromRadians(direction.omegaRadiansPerSecond * dtConstant));
    Twist2d twistVel = (new Pose2d()).log(robotPoseVel);

    ChassisSpeeds updatedChassisSpeed = new ChassisSpeeds(twistVel.dx /
        dtConstant, twistVel.dy / dtConstant,
        twistVel.dtheta / dtConstant);

    setChassisSpeeds(updatedChassisSpeed);

    // System.out.println((System.currentTimeMillis() - pastTime) / 1000);
    // pastTime = System.currentTimeMillis();
    // setChassisSpeeds(direction);

    SmartDashboard.putNumber("Robot Vel X", getRobotVelocity().vxMetersPerSecond);
    SmartDashboard.putNumber("Robot Vel Y", getRobotVelocity().vyMetersPerSecond);

    SmartDashboard.putNumber("Field Vel X", getFieldVelocity().vxMetersPerSecond);
    SmartDashboard.putNumber("Field Vel Y", getFieldVelocity().vyMetersPerSecond);
  }

  @Override
  public SwerveModulePosition[] getModulePositions() {
    return swerveDrive.getModulePositions();

  }

  @Override
  public void disabledBehavior() {

  }

  public void setAutoBuilder() {
    
    AutoBuilder.configureHolonomic(
                () -> getPoseEstimator().getCurrentPose(), // Pose2d supplier
               (Pose2d pose) -> getPoseEstimator().resetToPose(pose), // Pose2d consumer, used to reset odometry at the
                this::getRobotVelocity, //  ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
  }

  private int issueCheckCycles = 0;
  private int issueCount = 0;
  private static boolean useGlass = false;
  private Map<Integer, MechanismRoot2d> visualRoots = new HashMap<>();
  private Map<Integer, MechanismLigament2d> motorDials = new HashMap<>();
  private Map<Integer, MechanismLigament2d> absEncDials = new HashMap<>();
  private Map<Integer, MechanismLigament2d> expectDials = new HashMap<>();

  private int badConnections = 0;
  private double lowLimit = Units.inchesToMeters(-1);
  private double highXLimit = Units.feetToMeters(26);
  private double highYLimit = Units.feetToMeters(27);

  @Override
  public boolean hasIssues() {

    issueCheckCycles++;
    if (issueCheckCycles > 10) {
      issueCheckCycles = 0;

      boolean doesCanHaveIssues = RobotController.getCANStatus().transmitErrorCount
          + RobotController.getCANStatus().receiveErrorCount > 0;

      Translation2d currTranslation = getPoseEstimator().getCurrentPose().getTranslation();
      boolean positionOk = !DriverStation.isAutonomous()
          || (currTranslation.getX() >= lowLimit && currTranslation.getY() >= lowLimit) &&
              (currTranslation.getX() <= highXLimit && currTranslation.getY() <= highYLimit) &&
              (!Double.isNaN(currTranslation.getX()) &&
                  !Double.isNaN(currTranslation.getY()));

      if (doesCanHaveIssues) {
        badConnections++;
      } else {
        badConnections = 0;
      }
      if (!positionOk) {
        issueCount++;
      } else {
        issueCount = 0;
      }

      if (badConnections > 5) {
        System.err.println("********************************CAN is being flakey********************************");
      }
      if (issueCount > 5) {
        System.err
            .println("********************************Robot position is off field********************************");
      }

      return badConnections > 5 || !positionOk;
    }
    return false;
  }

  public void updateVisionMeasurements(Pose2d robotPose, double imageCaptureTime) {
    swerveDrive.addVisionMeasurement(robotPose, imageCaptureTime);
  }

  public static void useGlass(boolean shouldUseGlass) {
    useGlass = shouldUseGlass;
  }

  public void initGlassWidget() {
    SwerveModule[] modules = swerveDrive.getModules();
    visualRoots.put(0, mechVisual.getRoot("frontleft", 15, 45));
    visualRoots.put(1, mechVisual.getRoot("frontright", 45, 45));
    visualRoots.put(2, mechVisual.getRoot("backleft", 15, 15));
    visualRoots.put(3, mechVisual.getRoot("backright", 45, 15));
    for (int i = 0; i < modules.length; i++) {
      visualRoots.get(i).append(
          new MechanismLigament2d(i + "-vert", 10, 90, 6.0, new Color8Bit(50, 50, 50)));
      visualRoots.get(i).append(
          new MechanismLigament2d(i + "-hori", 10, 0, 6.0, new Color8Bit(50, 50, 50)));
      motorDials.put(i, visualRoots.get(i).append(
          new MechanismLigament2d(i + "-motor", 10.0, 90, 6.0, new Color8Bit(Color.kYellow))));
      absEncDials.put(i, visualRoots.get(i).append(
          new MechanismLigament2d(i + "-Abs", 10, 90, 6, new Color8Bit(Color.kBlue))));
      expectDials.put(i, visualRoots.get(i).append(
          new MechanismLigament2d(i + "-Exp", 10, 90, 6, new Color8Bit(Color.kRed))));
    }
  }

  public void updateGlassWidget() {
    SwerveModule[] modules = swerveDrive.getModules();
    for (int moduleKey = 0; moduleKey < modules.length; moduleKey++) {
      double turningDeg = modules[moduleKey].getRelativePosition();
      double absEncDeg = modules[moduleKey].getAbsolutePosition();
      SmartDashboard.putNumber("Motor Ang: " + moduleKey, turningDeg);
      SmartDashboard.putNumber("Abs Angle: " + moduleKey, absEncDeg);
      // This method will be called once per scheduler run
      absEncDials.get(moduleKey).setAngle(absEncDeg + 90);
      motorDials.get(moduleKey).setAngle(turningDeg + 90);
      motorDials.get(moduleKey).setLength(10 * modules[moduleKey].getAngleMotor().getVelocity() + 2);
      expectDials.get(moduleKey).setLength(10 * modules[moduleKey].getDriveMotor().getVelocity() + 2);
      expectDials.get(moduleKey).setAngle(modules[moduleKey].getState().angle.getDegrees() + 90);
    }
  }
}