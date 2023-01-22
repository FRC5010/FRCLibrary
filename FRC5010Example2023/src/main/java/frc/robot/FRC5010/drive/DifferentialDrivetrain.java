// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.FRC5010.Vision.VisionSystem;
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
  private List<Integer> motorPorts;
  private MotorController5010 left, right;
  private GenericEncoder leftEncoder, rightEncoder;
  private GenericGyro gyro;

  /**
   * Creates a new SkidSteerDrivetrain.
   * 
   * @param left  - requires the left motor as a template for the others
   * @param ports - a list of ports (assumes all ports are given)
   */
  public DifferentialDrivetrain(MotorController5010 left, List<Integer> ports,
      GenericGyro gyro, VisionSystem vision, Mechanism2d mechVisual, double kTrackwidthMeters) {
    super(mechVisual);
    assert (ports.size() == 4);
    this.motorPorts = ports;
    this.gyro = gyro;

    motorList = new ArrayList<>();
    this.left = left;
    motorList.add(left);
    MotorController5010 lMotor = left.duplicate(motorPorts.get(1));
    lMotor.setFollow(left);
    motorList.add(lMotor);

    this.right = left.duplicate(motorPorts.get(2));
    right.invert(true);
    MotorController5010 rMotor = right.duplicate(motorPorts.get(3));
    rMotor.setFollow(right);
    motorList.add(rMotor);

    leftEncoder = left.getMotorEncoder();
    rightEncoder = right.getMotorEncoder();

    diffKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

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

  private Persisted<Double> maxChassisVelocity = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_VELOCITY, Double.class);
  private Persisted<Double> maxChassisRotation = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_ROTATION, Double.class);

  @Override
  public void drive(ChassisSpeeds direction) {
    // WARNING: TODO: this may not be the 'best' way to convert chassis speeds to
    // throttles
    // For example - convert chassis speeds into left and right voltages based on
    // SysID character
    double throttle = Math.min(1,
        direction.vxMetersPerSecond / maxChassisVelocity.get());
    double rotation = Math.min(1,
        direction.omegaRadiansPerSecond / maxChassisRotation.getDouble());
    arcadeDrive(throttle, rotation);
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
    KvLinear = new Persisted<>(DriveConstantsDef.KV_DRIVE_LINEAR, Double.class);
    KaLinear = new Persisted<>(DriveConstantsDef.KA_DRIVE_LINEAR, Double.class);
    KvAngular = new Persisted<>(DriveConstantsDef.KV_DRIVE_ANGULAR, Double.class);
    KaAngular = new Persisted<>(DriveConstantsDef.KA_DRIVE_ANGULAR, Double.class);
    motorRotationsPerWheelRotation = new Persisted<>(DriveConstantsDef.MOTOR_ROT_PER_WHEEL_ROT, Double.class);
    kTrackwidthMeters = new Persisted<>(DriveConstantsDef.TRACK_WIDTH, Double.class);

    driveSim = new DifferentialDrivetrainSim(
        // Create a linear system from our identification gains.
        LinearSystemId.identifyDrivetrainSystem(KvLinear.getDouble(), KaLinear.getDouble(),
            KvAngular.getDouble(), KaAngular.getDouble()),
        DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
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
}
