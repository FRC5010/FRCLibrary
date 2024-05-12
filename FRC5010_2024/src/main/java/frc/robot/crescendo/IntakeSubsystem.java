// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.motors.PIDController5010;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.motors.PIDController5010.PIDControlType;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;

public class IntakeSubsystem extends GenericSubsystem {
  private MotorController5010 topIntakeMotor;
  private MotorController5010 bottomIntakeMotor;

  private GenericEncoder topEncoder;
  private GenericEncoder bottomEncoder;
  private PIDController5010 topPID;
  private PIDController5010 bottomPID;

  private SimpleMotorFeedforward feedforward;

  private final String TOP_SPEED = "Top Intake Speed";
  private final String BOTTOM_SPEED = "Bottom Intake Speed";
  private final String TOP_REFERENCE = "Top Intake Reference";
  private final String BOTTOM_REFERENCE = "Bottom Intake Reference";
  private final String IS_IN_PID_CONTROL = "Is PID Controlled?";

  private double topReference = 0.0;
  private double bottomReference = 0.0;

  private double topPreviousError;
  private double topPreviousTime;

  private double bottomPreviousError;
  private double bottomPreviousTime;

  private String INTAKE_SLOWDOWN = "Intake Slowdown";

  // Simulation Parts
  private Mechanism2d robotSim;

  private SimulatedEncoder simTopEncoder;
  private FlywheelSim simTop;
  private MechanismLigament2d topIntakeSim;


  private SimulatedEncoder simBottomEncoder;
  private FlywheelSim simBottom;
  private MechanismLigament2d bottomIntakeSim;

  private static enum IntakeState {
    Joystick, Velocity
  }

  private IntakeState intakeState = IntakeState.Joystick;

  // NetworkTable names

  private final double topConversionFactor = 1.0;
  private final double bottomConversionFactor = 1.0;
  private SimpleMotorFeedforward topFeedFwd;
  private SimpleMotorFeedforward bottomFeedFwd;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(MotorController5010 top, MotorController5010 bottom, Mechanism2d mechSim) {
    topFeedFwd = new SimpleMotorFeedforward(0.29808, 0.12554, 0.0032103);
    bottomFeedFwd = new SimpleMotorFeedforward(0.21824, 0.21824, 0.0033022);

    values.declare(TOP_SPEED, 0.0);
    values.declare(BOTTOM_SPEED, 0.0);
    values.declare(INTAKE_SLOWDOWN, 0.8);
    values.declare(TOP_REFERENCE, 0.0);
    values.declare(BOTTOM_REFERENCE, 0.0);
    values.declare(IS_IN_PID_CONTROL, false);

    topIntakeMotor = top;
    topEncoder = topIntakeMotor.getMotorEncoder();

    bottomIntakeMotor = bottom;
    bottomEncoder = bottomIntakeMotor.getMotorEncoder();

    topEncoder.setPositionConversion(topConversionFactor);
    bottomEncoder.setPositionConversion(bottomConversionFactor);

    topPID = topIntakeMotor.getPIDController5010();
    bottomPID = bottomIntakeMotor.getPIDController5010();

    topPID.setP(0.00011926);
    topPID.setI(0.0);
    topPID.setD(0.0);

    bottomPID.setP(1.9961E-05);
    bottomPID.setI(0.0);
    bottomPID.setD(0.0);


    simTop = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
    simBottom = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
    robotSim = mechSim;

    topIntakeSim = robotSim.getRoot("Top Intake", 0.10, 0.35).append(new MechanismLigament2d("Top Intake Dial", 0.1, 0, 5.0, new Color8Bit(Color.kFirstBlue)));


    bottomIntakeSim = robotSim.getRoot("Bottom Intake", 0.10, 0.2).append(new MechanismLigament2d("Bottom Intake Dial", 0.1, 0, 5.0, new Color8Bit(Color.kFirstRed)));




    // Sim Init
    simTopEncoder = new SimulatedEncoder(24, 25);
    simBottomEncoder = new SimulatedEncoder(26, 27);
  }

  public double getTopFeedFwdVoltage(double velocity) {
    return topFeedFwd.calculate(velocity);
  }

  public double getBottomFeedFwdVoltage(double velocity) {
    return bottomFeedFwd.calculate(velocity);
  }

  public Command getBottomSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(SystemIdentification.rpmSysIdRoutine(bottomIntakeMotor,
        bottomIntakeMotor.getMotorEncoder(), "Bottom Motor", this), 10, 3, 3);
  }

  public Command getTopSysIdRoutineCommand() {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(topIntakeMotor, topIntakeMotor.getMotorEncoder(), "Top Motor", this), 10, 3,
        3);
  }

  public void stateMachine(double joystick) {
    double slowed = joystick * values.getDouble(INTAKE_SLOWDOWN);
    switch (intakeState) {
      case Joystick:
        if (joystick == 0) {
          intakeState = IntakeState.Velocity;
          setReference(0, 0);
          topRunToReference();
          bottomRunToReference();
        } else {
          setIntakeSpeed(slowed, slowed);
        }
        break;
    
      case Velocity:
        if (joystick != 0) {
          intakeState = IntakeState.Joystick;
          setIntakeSpeed(slowed, slowed);
        } else {
          topRunToReference();
          bottomRunToReference();
        }
        break;
    }
  }

  public void setIntakeSpeed(double topSpeed, double bottomSpeed) {
    topIntakeMotor.set(topSpeed);
    bottomIntakeMotor.set(bottomSpeed);
  }

  public double getTopIntakeVelocity() {
    return Robot.isReal() ? topEncoder.getVelocity() : simTopEncoder.getVelocity();
  }

  public double getBottomIntakeVelocity() {
    return Robot.isReal() ? bottomEncoder.getVelocity() : simBottomEncoder.getVelocity();
  }

  public double getTopReference() {
    return topReference;
  }

  public double getBottomReference() {
    return bottomReference;
  }

  public void setReference(double topSpeed, double bottomSpeed) {
    intakeState = IntakeState.Velocity;
    topReference = topSpeed;
    bottomReference = bottomSpeed;
  }

  public void topRunToReference() {
     double feedForward = getTopFeedFwdVoltage(getTopReference());

    if (Robot.isReal()) {
      topPID.setReference(getTopReference(), PIDControlType.VELOCITY, feedForward);
      return;
    }


    double currentError = getTopReference() - getTopIntakeVelocity();
    double currentTime = RobotController.getFPGATime() / 1E6;
    double errorRate = (currentError - topPreviousError) / (currentTime - topPreviousTime);
    double voltage = currentError * topPID.getP() + (errorRate * topPID.getD());
  
    topIntakeMotor.set(feedForward/ RobotController.getBatteryVoltage() + voltage);
  
    topPreviousError = currentError;
    topPreviousTime = currentTime;
  }

  public void bottomRunToReference() {
    double feedForward = getBottomFeedFwdVoltage(getBottomReference());

    if (Robot.isReal()) {
      bottomPID.setReference(getBottomReference(), PIDControlType.VELOCITY, feedForward);
      return;
    }

    double currentError = getBottomReference() - getBottomIntakeVelocity();
    double currentTime = RobotController.getFPGATime() / 1E6;
    double errorRate = (currentError - bottomPreviousError) / (currentTime - bottomPreviousTime);
    double voltage = currentError * bottomPID.getP() + (errorRate * bottomPID.getD());
    
    bottomIntakeMotor.set(feedForward/ RobotController.getBatteryVoltage() + voltage);

    bottomPreviousError = currentError;
    bottomPreviousTime = currentTime;
  }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    topIntakeSim.setAngle(topIntakeMotor.get() * 180);
    bottomIntakeSim.setAngle(bottomIntakeMotor.get()* 180);

    values.set(TOP_SPEED, topEncoder.getVelocity());
    values.set(BOTTOM_SPEED, bottomEncoder.getVelocity());
    values.set(TOP_REFERENCE, getTopReference());
    values.set(BOTTOM_REFERENCE, getBottomReference());
    values.set(IS_IN_PID_CONTROL, intakeState == IntakeState.Velocity ? true : false);

  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    simTop.setInput(topIntakeMotor.get() * RobotController.getBatteryVoltage());
    simBottom.setInput(bottomIntakeMotor.get() * RobotController.getBatteryVoltage());
    
    // Next, we update it. The standard loop time is 20ms.
    simTop.update(0.020);
    simBottom.update(0.020);


    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    

    //simPivotArm.setAngle(Units.degreesToRadians(pivotSim.getAngleRads()));


    
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(simTop.getCurrentDrawAmps(), simBottom.getCurrentDrawAmps()));

  }
}
