// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.swerve;

import java.io.File;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.pose.YAGSLSwervePose;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.commands.JoystickToSwerve;
import frc.robot.commands.TeleopDrive;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveKinematics2;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

/** Add your docs here. */
public class YAGSLSwerveDrivetrain extends SwerveDrivetrain {
  private SwerveDrive swerveDrive;
  private ChassisSpeeds chassisSpeeds;
  private PowerDistribution powerDistributionHub;
  private GenericGyro gyro;

  public YAGSLSwerveDrivetrain(Mechanism2d mechVisual, GenericGyro gyro, SwerveConstants swerveConstants,
      String swerveType, VisionSystem visionSystem) {
    super(mechVisual, gyro, swerveConstants);
    this.gyro = gyro;
    try {
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), swerveType);
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive();
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }
    poseEstimator = new DrivetrainPoseEstimator(new YAGSLSwervePose(gyro, this), visionSystem);
    setDrivetrainPoseEstimator(poseEstimator);

    // swerveDrive.setModuleStates();
    // powerDistributionHub = new PowerDistribution(1, ModuleType.kRev);
  }

  public Command createDefaultCommand(Controller driverXbox) {
    // System.out.println("brrrrr");
    Supplier<Double> leftX = () -> driverXbox.getLeftXAxis();
    Supplier<Double> leftY = () -> driverXbox.getLeftYAxis();
    Supplier<Double> rightX = () -> driverXbox.getRightXAxis();
    Supplier<Boolean> isFieldOriented = () -> isFieldOrientedDrive;

    return new JoystickToSwerve(this, leftY, leftX, rightX, isFieldOriented);
    // return new TeleopDrive(this,
    // () -> driverXbox.getLeftYAxis(),
    // () -> driverXbox.getLeftXAxis(),
    // () -> driverXbox.getRightXAxis(),
    // () -> isFieldOrientedDrive, true, true);
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
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true
   *                      to disable closed-loop.
   */

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
  }

  @Override
  public void stop() {
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    poseEstimator.update();

    // SmartDashboard.putNumber("FL Wheel Current",
    // powerDistributionHub.getCurrent(1));

    // SmartDashboard.putNumber("FR Wheel Current",
    // powerDistributionHub.getCurrent(18));

    // SmartDashboard.putNumber("BL Wheel Current",
    // powerDistributionHub.getCurrent(3));

    // SmartDashboard.putNumber("BR Wheel Current",
    // powerDistributionHub.getCurrent(15));
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveKinematics2} of the swerve drive.
   */
  public SwerveKinematics2 getKinematics() {
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
    swerveDrive.setGyro(new Rotation3d(0, 0, initialHolonomicPose.getRotation().getRadians()));
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

  public void updateVisionMeasurements(Pose2d robotPose, double imageCaptureTime) {
    swerveDrive.addVisionMeasurement(robotPose, imageCaptureTime, true, 1);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  @Override
  public void resetEncoders() {
    swerveDrive.resetEncoders();
    swerveDrive.synchronizeModuleEncoders();
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
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
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
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp(), false, 1);
  }

  @Override
  public void drive(ChassisSpeeds direction) {

    Translation2d currTranslation = getPoseEstimator().getCurrentPose().getTranslation();
    if (!DriverStation.isAutonomous() ||
        (currTranslation.getX() >= 0 && currTranslation.getY() >= 0) &&
            (currTranslation.getX() <= 26 && currTranslation.getY() <= 27) &&
            (!Double.isNaN(currTranslation.getX()) &&
                !Double.isNaN(currTranslation.getY()))) {

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

    } else {
      System.err.println("******CRITICAL ERROR******: Pose Outside of Bounds " + currTranslation);
      setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

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

  public BaseAutoBuilder setAutoBuilder(Map<String, Command> eventMap) {
    return new SwerveAutoBuilder(
        () -> getPoseEstimator().getCurrentPose(), // Pose2d supplier
        (Pose2d pose) -> getPoseEstimator().resetToPose(pose), // Pose2d consumer, used to reset odometry at the
                                                               // beginning of auto
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
                                         // PID controllers)
        new PIDConstants(2.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
                                         // controller)
        swerveDrive::setChassisSpeeds, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color.
              // Optional, defaults to true
        this // The drive subsystem. Used to properly set the requirements of path following
             // commands
    );
  }
}