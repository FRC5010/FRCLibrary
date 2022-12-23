// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.Impl.SimulatedEncoder;
import frc.robot.FRC5010.Impl.SimulatedGyro;

/** Add your docs here. */
public abstract class GenericSwerveModule extends SubsystemBase {
    private MechanismRoot2d visualRoot;
    private final MechanismLigament2d motorDial;
    private final MechanismLigament2d absEncDial;
    private final MechanismLigament2d expectDial;
    private final String moduleKey;
    protected SwerveModuleState state;
    protected GenericEncoder driveEncoder = new SimulatedEncoder(0, 1); 
    protected GenericEncoder turnEncoder = new SimulatedEncoder(2, 3);
    protected GenericGyro gyro = new SimulatedGyro();

    public GenericSwerveModule(MechanismRoot2d visualRoot, String key) {
        this.visualRoot = visualRoot;
        this.moduleKey = key;
        motorDial = visualRoot.append(
                new MechanismLigament2d(moduleKey, 10.0, 90, 6.0, new Color8Bit(Color.kYellow)));
        absEncDial = visualRoot.append(
                new MechanismLigament2d(moduleKey + "Abs", 10, 90, 6, new Color8Bit(Color.kBlue)));
        expectDial = visualRoot.append(
                new MechanismLigament2d(moduleKey + "Exp", 10, 90, 6, new Color8Bit(Color.kRed)));
        initSimulation();        
    }

    public abstract SwerveModuleState getState();

    public abstract void resetEncoders();

    public abstract double getTurningPosition();

    public abstract double getAbsoluteEncoderRad();

    public abstract double getTurningMotor();

    public abstract double getDriveMotor();

    @Override
    public void periodic() {
        double turningDeg = Units.radiansToDegrees(getTurningPosition()) + 90;
        double absEncDeg = Units.radiansToDegrees(getAbsoluteEncoderRad()) + 90;
        SmartDashboard.putNumber("Motor Ang: " + moduleKey, turningDeg);
        SmartDashboard.putNumber("Abs Angle: " + moduleKey, absEncDeg);
        SmartDashboard.putNumber("Abs Rads: " + moduleKey, getAbsoluteEncoderRad());
        // This method will be called once per scheduler run
        absEncDial.setAngle(absEncDeg);
        motorDial.setAngle(turningDeg);
        motorDial.setLength(50 * getTurningMotor());
        expectDial.setLength(50 * getDriveMotor());
        expectDial.setAngle(state.angle.getDegrees() + 90);
    }

    // Simulation
    // Create our feedforward gain constants (from the identification tool)
    public static final double kvVoltSecondsPerMeter = 1.3984; // 1.3568; // 1.3795; //1.1055;
    public static final double kaVoltSecondsSquaredPerMeter = 0.27137; // 0.24208; //0.24467;//0.23628;
    public static final double motorRotationsPerWheelRotation = 10.71; // 0.24208; //0.24467;//0.23628;

    static final double KvLinear = kvVoltSecondsPerMeter;
    static final double KaLinear = kaVoltSecondsSquaredPerMeter;
    static final double KvAngular = 1.5;
    static final double KaAngular = 0.3;
    // Create the simulation model of our drivetrain.
    public static DifferentialDrivetrainSim driveSim;
    
    public void initSimulation() {
        driveSim = new DifferentialDrivetrainSim(
                // Create a linear system from our identification gains.
                 LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
                 DCMotor.getNEO(1), // 2 NEO motors on each side of the drivetrain.
                 motorRotationsPerWheelRotation, // 10.71:1 gearing reduction.
                 0.02,
                 KitbotWheelSize.kSixInch.value, // The robot uses 3" radius wheels.

                // The standard deviations for measurement noise:
                // x and y: 0.001 m
                // heading: 0.001 rad
                // l and r velocity: 0.1 m/s
                // l and r position: 0.005 m
                 VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    }

    @Override
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        driveSim.setInputs(
                 getDriveMotor() * RobotController.getInputVoltage(),
                 getDriveMotor() * RobotController.getInputVoltage());
        driveSim.update(0.02);

        driveEncoder.setPosition(driveSim.getLeftPositionMeters());
        driveEncoder.setRate(driveSim.getLeftVelocityMetersPerSecond());
        // rightEncoder.setPosition(Drive.m_driveSim.getRightPositionMeters());
        // rightEncoder.setRate(Drive.m_driveSim.getRightVelocityMetersPerSecond());
        gyro.setAngle(turnEncoder.getPosition());
    }
}
