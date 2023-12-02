// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.DrivePorts;
import frc.robot.FRC5010.constants.GenericDrivetrainConstants;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.drive.pose.DifferentialPose;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.mechanisms.DriveConstantsDef;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.sensors.encoder.GenericEncoder;
import frc.robot.FRC5010.sensors.encoder.SimulatedEncoder;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;

public class DifferentialDrivetrain extends GenericDrivetrain {
  private List<MotorController5010> motorList;
  private DifferentialDrive diffDrive;
  private DifferentialDriveKinematics diffKinematics;
  private List<? extends DrivePorts> motorPorts;
  private MotorController5010 left, right;
  private GenericEncoder leftEncoder, rightEncoder;
  private GenericGyro gyro;
  private GenericDrivetrainConstants driveConstants;
  private DCMotor dcMotor = DCMotor.getRomiBuiltIn(2);
  /**
   * Creates a new SkidSteerDrivetrain.
   * 
   * @param left  - requires the left motor as a template for the others
   * @param ports - a list of ports (assumes all ports are given)
   */
  public DifferentialDrivetrain(MotorController5010 left, List<DrivePorts> ports,
      GenericGyro gyro, VisionSystem vision, GenericDrivetrainConstants driveConstants,
      Mechanism2d mechVisual) {
    super(mechVisual);
    this.motorPorts = ports;
    this.gyro = gyro;
    this.driveConstants = driveConstants;

    motorList = new ArrayList<>();
    this.left = left;
    motorList.add(left);
    if (ports.size() == 4) {
      MotorController5010 lMotor = left.duplicate(motorPorts.get(1).getDrivePort());
      lMotor.setFollow(left);
      motorList.add(lMotor);
    }

    this.right = left.duplicate(motorPorts.get(2).getDrivePort());
    right.invert(true);
    if (ports.size() == 4) {
      MotorController5010 rMotor = right.duplicate(motorPorts.get(3).getDrivePort());
      rMotor.setFollow(right);
      motorList.add(rMotor);
    }
    init(vision);
  }

  public DifferentialDrivetrain(MotorController5010 left, MotorController5010 right,
      GenericGyro gyro, VisionSystem vision, GenericDrivetrainConstants driveConstants,
      List<? extends DrivePorts> ports,
      Mechanism2d mechVisual) {
    super(mechVisual);
    this.gyro = gyro;
    this.driveConstants = driveConstants;
    motorPorts = ports;
    this.left = left;
    this.right = right;
    setMaxChassisVelocity(driveConstants.getkPhysicalMaxSpeedMetersPerSecond());
    setMaxRotationVelocity(driveConstants.getkPhysicalMaxAngularSpeedRadiansPerSecond());
    isFieldOrientedDrive = false;
    init(vision);
  }

  protected void init(VisionSystem vision) {
    leftEncoder = left.getMotorEncoder();
    rightEncoder = right.getMotorEncoder();

    diffKinematics = new DifferentialDriveKinematics(driveConstants.getTrackWidth());

    if (RobotBase.isSimulation()) {
      initSimulation();
    }

    setDrivetrainPoseEstimator(
        new DrivetrainPoseEstimator(
            new DifferentialPose(diffKinematics, gyro, leftEncoder, rightEncoder), vision));
    diffDrive = new DifferentialDrive(left.getMotor(), right.getMotor());
  }

  public void arcadeDrive(double throttle, double steer) {
    diffDrive.arcadeDrive(throttle, steer);
  }

  @Override
  public void drive(ChassisSpeeds direction) {
    // WARNING: TODO: this may not be the 'best' way to convert chassis speeds to
    // throttles
    // For example - convert chassis speeds into left and right voltages based on
    // SysID character
    SmartDashboard.putNumber("vxMpS", direction.vxMetersPerSecond);
    SmartDashboard.putNumber("omegaRpS", direction.omegaRadiansPerSecond);
    double throttle = Math.min(1,
        direction.vxMetersPerSecond / maxChassisVelocity);
    double rotation = Math.min(1,
        direction.omegaRadiansPerSecond / maxRotationVelocity);
    SmartDashboard.putNumber("throttle", throttle);
    SmartDashboard.putNumber("rotation", rotation);
    arcadeDrive(throttle, rotation);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * 
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(-rightVolts); // We invert this to maintain +ve = forward
    diffDrive.feed();
  }

  // Simulation
  // Create our feedforward gain constants (from the identification tool)
  private Persisted<Double> KvLinear;
  private Persisted<Double> KaLinear;
  private Persisted<Double> KvAngular;
  private Persisted<Double> KaAngular;
  private Persisted<Double> motorRotationsPerWheelRotation;
  private Persisted<Double> kTrackwidthMeters;

  // Create the simulation model of our drivetrain.
  public static DifferentialDrivetrainSim driveSim;

  public void initSimulation() {
    KvLinear = new Persisted<>(DriveConstantsDef.KV_DRIVE_LINEAR, motorPorts.get(0).getMotorConstants().getkV());
    KaLinear = new Persisted<>(DriveConstantsDef.KA_DRIVE_LINEAR, motorPorts.get(0).getMotorConstants().getkA());
    motorRotationsPerWheelRotation = new Persisted<>(DriveConstantsDef.MOTOR_ROT_PER_WHEEL_ROT, 10.71);

    KvAngular = new Persisted<>(DriveConstantsDef.KV_DRIVE_ANGULAR, 0.5 / 180);
    KaAngular = new Persisted<>(DriveConstantsDef.KA_DRIVE_ANGULAR, 0.03);
    motorRotationsPerWheelRotation = new Persisted<>(DriveConstantsDef.MOTOR_ROT_PER_WHEEL_ROT, 10.71);
    kTrackwidthMeters = new Persisted<>(DriveConstantsDef.TRACK_WIDTH, Double.class);

    driveSim = new DifferentialDrivetrainSim(
        // Create a linear system from our identification gains.
        LinearSystemId.identifyDrivetrainSystem(KvLinear.getDouble(), KaLinear.getDouble(),
            KvAngular.getDouble(), KaAngular.getDouble()),
        DCMotor.getRomiBuiltIn(2), // 2 NEO motors on each side of the drivetrain.
        motorRotationsPerWheelRotation.getDouble(), // 10.71:1 gearing reduction.
        kTrackwidthMeters.getDouble(), // The track width is 0.616 meters.
        KitbotWheelSize.kSixInch.value, // The robot uses 3" radius wheels.

        // The standard deviations for measurement noise:
        // x and y: 0.001 m
        // heading: 0.001 rad
        // l and r velocity: 0.1 m/s
        // l and r position: 0.005 m
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    leftEncoder = new SimulatedEncoder(10, 11);
    rightEncoder = new SimulatedEncoder(12, 13);
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    driveSim.setInputs(
        left.get() * RobotController.getInputVoltage(),
        right.get() * RobotController.getInputVoltage());
    driveSim.update(0.02);

    leftEncoder.setPosition(driveSim.getLeftPositionMeters());
    leftEncoder.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoder.setPosition(driveSim.getRightPositionMeters());
    rightEncoder.setRate(driveSim.getRightVelocityMetersPerSecond());
    gyro.setAngle(-driveSim.getHeading().getDegrees());
  }

  @Override
  public BaseAutoBuilder setAutoBuilder(Map<String, Command> eventMap) {
    // TODO Auto-generated method stub
    return null;
  }

  /**
   * Returns the current wheel speeds of the robot.
   * 
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left.getMotorEncoder().getVelocity(),
        right.getMotorEncoder().getVelocity());
  }

  /**
   * Generate a trajectory following Ramsete command
   * 
   * This is very similar to the WPILib RamseteCommand example. It uses
   * constants defined in the Constants.java file. These constants were
   * found empirically by using the frc-characterization tool.
   * 
   * @return A SequentialCommand that sets up and executes a trajectory following
   *         Ramsete command
   */
  private Command generateRamseteCommand(Trajectory trajectory) {
    return generateRamseteCommand(trajectory, true, true);
  }

  private Command generateRamseteCommand(Trajectory trajectory, boolean reset, boolean stop) {

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        () -> getPoseEstimator().getCurrentPose(),
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        this::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        this::tankDriveVolts,
        this);

    poseEstimator.resetToPose(trajectory.getInitialPose());

    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(
        () -> {
          if (reset)
            poseEstimator.resetToPose(trajectory.getInitialPose());
        }, this)

        // next, we run the actual ramsete command
        .andThen(ramseteCommand)

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> {
          if (stop)
            tankDriveVolts(0, 0);
        }, this));
  }

}
