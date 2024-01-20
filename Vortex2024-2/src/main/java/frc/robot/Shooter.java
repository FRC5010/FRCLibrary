// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

public class Shooter extends SubsystemBase {
  private static final int kMotorPort = 18; // Change back to 2
  private static final int kMotorPort2 = 19; // Change back to 1
  private static final int kMotorPort3 = 3; // Change back to 15

  // private static final double BottomKs = 0.098517;
  // private static final double BottomKv = 0.45615 / 60.0 / (12.0 - BottomKs);
  // private static final double BottomKa = 0.059296 / 60.0 / (12.0 - BottomKs);
  // private static final double BottomKp = 0.038178;

  // private static final double TopKs = 0.028379;
  // private static final double TopKa = 0.087259 / 60.0 / (12.0 - TopKs);
  // private static final double TopKv = 0.43275 / 60.0 / (12.0 - TopKs);
  // private static final double TopKp = 1.5572E-07;

  // Comp Bot Intake motors
  private static final double TopKs = 0.21531;
  private static final double TopKv = 0.002122 / 12.0;
  private static final double TopKa = 0.00017499 / 12.0;
  private static final double TopKp = 10.1707E-5;

  private static final double BottomKs = 0.2326;
  private static final double BottomKv = 0.0021369 / 12.0;
  private static final double BottomKa = 0.00019341 / 12.0;
  private static final double BottomKp = 10.3633E-5;

  private CANSparkMax top_motor; // Change to CANSparkMax
  private CANSparkMax bottom_motor; // Change to CANSparkMax
  private CANSparkMax feeder_motor; // Change to CANSparkMax
  private RelativeEncoder top_encoder;
  private RelativeEncoder bottom_encoder;
  private RelativeEncoder feeder_encoder;
  private SparkPIDController top_pid;
  private SparkPIDController bot_pid;
  private SparkPIDController feed_pid;

  private SysIdRoutine sysIdRoutine;
  // private SysIdRoutine bottomRoutine;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_distance = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  /** Creates a new Shooter. */
  public Shooter() {
    SmartDashboard.putNumber("Top Motor", 0.0);
    SmartDashboard.putNumber("Down Motor", 0.0);
    SmartDashboard.putNumber("Feeder Motor", 0.0);
    top_motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    top_motor.restoreFactoryDefaults();
    top_encoder = top_motor.getEncoder();

    bottom_motor = new CANSparkMax(kMotorPort2, MotorType.kBrushless);
    bottom_motor.restoreFactoryDefaults();
    bottom_encoder = bottom_motor.getEncoder();

    feeder_motor = new CANSparkMax(kMotorPort3, MotorType.kBrushless);
    feeder_motor.restoreFactoryDefaults();
    feeder_motor.setInverted(true);
    feeder_encoder = feeder_motor.getEncoder();

    top_encoder.setPosition(0);
    bottom_encoder.setPosition(0);
    feeder_encoder.setPosition(0);

    top_motor.set(0);

    top_motor.setIdleMode(IdleMode.kCoast);
    bottom_motor.setIdleMode(IdleMode.kCoast);
    feeder_motor.setIdleMode(IdleMode.kCoast);

    // top_encoder.setPositionConversionFactor((Math.PI * Units.inchesToMeters(3)));
    // bottom_encoder.setPositionConversionFactor((Math.PI * Units.inchesToMeters(3)));
    // feeder_encoder.setPosition(Math.PI * Units.inchesToMeters(2));
    // top_encoder.setVelocityConversionFactor((Math.PI * Units.inchesToMeters(3) / 60.0));
    // bottom_encoder.setVelocityConversionFactor((Math.PI * Units.inchesToMeters(3) / 60.0));
    // feeder_encoder.setVelocityConversionFactor((Math.PI * Units.inchesToMeters(2) / 60));

   // setSysIdRoutine(top_motor, "top-motor");
    setSysIdRoutine(bottom_motor, "motor");

    top_pid = top_motor.getPIDController();
    bot_pid = bottom_motor.getPIDController();
    feed_pid = feeder_motor.getPIDController();
    top_pid.setP(TopKp);
    top_pid.setD(0);
    top_pid.setFF(TopKv);
    top_pid.setI(0);

    bot_pid.setP(BottomKp);
    bot_pid.setD(0);
    bot_pid.setFF(BottomKv);
    bot_pid.setI(0);

    feed_pid.setP(0);
    feed_pid.setD(0);
    feed_pid.setFF(0);
    feed_pid.setI(0);
  }

  public double deadzone(double value, double deadzone) {
    if (Math.abs(value) < deadzone)
      return 0.0;

    return value;
  }

  public void invertMotors(boolean invert) {
    if (invert != top_motor.getInverted())
      top_motor.setInverted(invert);
    if (invert != bottom_motor.getInverted())
      bottom_motor.setInverted(invert);
  }

  public void stopMotors() {
    top_motor.set(0);
    bottom_motor.set(0);
    feeder_motor.set(0);
    top_encoder.setPosition(0);
    bottom_encoder.setPosition(0);
    feeder_encoder.setPosition(0);
  }

  public void runMotors(CommandXboxController joystick) {
    double Jpower = -deadzone(joystick.getRawAxis(1), 0.05);
    if (joystick.rightBumper().getAsBoolean()) {
      top_motor.set(SmartDashboard.getNumber("Top Motor", 0.0));
      bottom_motor.set(SmartDashboard.getNumber("Down Motor", 0.0));
      feeder_motor.set(SmartDashboard.getNumber("Feeder Motor", 0.0));
    } else if (joystick.leftBumper().getAsBoolean()) {
      top_pid.setReference(SmartDashboard.getNumber("Top Motor", 0.0), ControlType.kVelocity, 0, TopKs);
      bot_pid.setReference(SmartDashboard.getNumber("Down Motor", 0.0), ControlType.kVelocity, 0, BottomKs);
      feeder_motor.set(SmartDashboard.getNumber("Feeder Motor", 0.0));
    } else {
      top_motor.set(Jpower);
      bottom_motor.set(Jpower);
      feeder_motor.set(Jpower);
    }
  }

  public void setSysIdRoutine(CANSparkMax motor, String motorName) {
    RelativeEncoder encoder = motor.getEncoder();
    sysIdRoutine = new SysIdRoutine(new Config(), new SysIdRoutine.Mechanism(
        (Measure<Voltage> voltage) -> motor.set(voltage.in(Volts) / RobotController.getBatteryVoltage()),
        log -> {
          log.motor(motorName)
              .voltage(
                  m_appliedVoltage.mut_replace(
                      motor.get() * RobotController.getBatteryVoltage(), Volts))
              .angularPosition(m_distance.mut_replace(encoder.getPosition(), Rotations))
              .angularVelocity(m_velocity.mut_replace(encoder.getVelocity(), RotationsPerSecond));
        }, this));
  }

  public Command topSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command topSysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Encoder", top_encoder.getPosition());
    SmartDashboard.putNumber("Top Velocity", top_encoder.getVelocity());
    SmartDashboard.putNumber("Top Voltage", top_motor.get());
    SmartDashboard.putNumber("Batt Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Encoder Counts Per Revolution", top_encoder.getCountsPerRevolution());
    SmartDashboard.putNumber("Bot Encoder", bottom_encoder.getPosition());
    SmartDashboard.putNumber("Bot Velocity", bottom_encoder.getVelocity());
    SmartDashboard.putNumber("Bot Voltage", bottom_motor.get() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Feed Encoder", feeder_encoder.getPosition());
    SmartDashboard.putNumber("Feed Velocity", feeder_encoder.getVelocity());
  }
}
