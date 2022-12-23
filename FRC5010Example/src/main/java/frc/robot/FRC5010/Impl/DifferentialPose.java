// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.Impl;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.FRC5010.GenericEncoder;
import frc.robot.FRC5010.GenericGyro;
import frc.robot.FRC5010.GenericPose;

/** Add your docs here. */
public class DifferentialPose extends GenericPose {
    private GenericEncoder leftEncoder;
    private GenericEncoder rightEncoder;
    private final DifferentialDrivePoseEstimator poseEstimator;

    public DifferentialPose(GenericGyro gyro, GenericEncoder leftEncoder, GenericEncoder rightEncoder) {
        super(gyro);
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        poseEstimator = new DifferentialDrivePoseEstimator(
                getGyroRotation2d(),
                new Pose2d(),
                stateStdDevs,
                localMeasurementStdDevs,
                visionMeasurementStdDevs);
        initSimulation();
    }

    public void setupSimulator(double kV, double kA, double kVAngular, double kAAngular, 
        double gearing, double trackWidth, MotorController lDrive, MotorController rDrive) {
        KvLinear = kV;
        KaLinear = kA;
        KvAngular = kVAngular;
        KaAngular = kAAngular;
        motorRotationsPerWheelRotation = gearing;
        kTrackwidthMeters = trackWidth;
        this.lDrive = lDrive;
        this.rDrive = rDrive; 
    }

    @Override
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void updateVision(Pose2d robotPose, double imageCaptureTime) {
        poseEstimator.resetPosition(robotPose, robotPose.getRotation());
        // m_poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
    }

    public void updatePhysics() {
        DifferentialDriveWheelSpeeds actWheelSpeeds = new DifferentialDriveWheelSpeeds(
                leftEncoder.getVelocity(), rightEncoder.getVelocity());
        double leftDist = leftEncoder.getPosition();
        double rightDist = rightEncoder.getPosition();
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation2d(),
                actWheelSpeeds, leftDist, rightDist);
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }
    
    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(),rightEncoder.getVelocity());
    }

    public void resetToPose(Pose2d pose) {
        poseEstimator.resetPosition(pose, getGyroRotation2d());
    }

      // Simulation
  // Create our feedforward gain constants (from the identification tool)
  private double KvLinear = 0;
  private double KaLinear = 0;
  private double KvAngular = 0;
  private double KaAngular = 0;
  private double motorRotationsPerWheelRotation = 0;
  private double kTrackwidthMeters = 0;
  private MotorController lDrive;
  private MotorController rDrive;

  // Create the simulation model of our drivetrain.
  public static DifferentialDrivetrainSim driveSim;
  public void initSimulation() {
    driveSim = new DifferentialDrivetrainSim(
      // Create a linear system from our identification gains.
      LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
      DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
      motorRotationsPerWheelRotation,  // 10.71:1 gearing reduction.
      kTrackwidthMeters, // The track width is 0.616 meters.
      KitbotWheelSize.kSixInch.value, // The robot uses 3" radius wheels.
    
      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  }
  
    /** Update our simulation. This should be run every robot loop in simulation. */
    @Override
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        driveSim.setInputs(
                lDrive.get() * RobotController.getInputVoltage(),
                rDrive.get() * RobotController.getInputVoltage());
        driveSim.update(0.02);

        leftEncoder.setPosition(driveSim.getLeftPositionMeters());
        leftEncoder.setRate(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoder.setPosition(driveSim.getRightPositionMeters());
        rightEncoder.setRate(driveSim.getRightVelocityMetersPerSecond());
        gyro.setAngle(-driveSim.getHeading().getDegrees());
    }
}
