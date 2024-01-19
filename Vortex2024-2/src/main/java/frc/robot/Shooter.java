// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
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
  private static final int kMotorPort = 2;
  private static final int kMotorPort2 = 1;

  private CANSparkFlex top_motor;
  private CANSparkFlex bottom_motor;
  private RelativeEncoder top_encoder;
  private RelativeEncoder bottom_encoder;
  private SparkPIDController top_pid;
  private SparkPIDController bot_pid;

  private SysIdRoutine topRoutine;
  //private SysIdRoutine bottomRoutine;

  private double topKs = 0;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  /** Creates a new Shooter. */
  public Shooter() {
    SmartDashboard.putNumber("Top Motor", 0.0);
    SmartDashboard.putNumber("Down Motor", 0.0);
    top_motor = new CANSparkFlex(kMotorPort, MotorType.kBrushless);
    top_motor.restoreFactoryDefaults();
    top_encoder = top_motor.getEncoder();
    bottom_motor = new CANSparkFlex(kMotorPort2, MotorType.kBrushless);
    bottom_motor.restoreFactoryDefaults();
    bottom_encoder = bottom_motor.getEncoder();
    // top_motor.setInverted(true);
    // bottom_motor.setInverted(true);
    top_encoder.setPosition(0);
    bottom_encoder.setPosition(0);
    // bottomRoutine = new SysIdRoutine(new Config(), new
    // SysIdRoutine.Mechanism(this::bottomVoltageDrive, log -> {
    // log.motor("bottom motor")
    // .voltage(
    // m_appliedVoltage.mut_replace(
    // bottom_motor.get() * RobotController.getBatteryVoltage(), Volts))
    // .linearPosition(m_distance.mut_replace(bottom_encoder.getPosition(), Meters))
    // .linearVelocity(
    // m_velocity.mut_replace(bottom_encoder.getVelocity(), MetersPerSecond));
    // }, this));
    topRoutine = new SysIdRoutine(new Config(), new SysIdRoutine.Mechanism(this::topVoltageDrive, log -> {
      log.motor("top motor")
          .voltage(
              m_appliedVoltage.mut_replace(
                  top_motor.get() * RobotController.getBatteryVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(top_encoder.getPosition(), Meters))
          .linearVelocity(
              m_velocity.mut_replace(top_encoder.getVelocity(), MetersPerSecond));
    }, this));

    top_motor.set(0);

    top_motor.setIdleMode(IdleMode.kCoast);
    bottom_motor.setIdleMode(IdleMode.kCoast);

    top_encoder.setPositionConversionFactor((Math.PI * Units.inchesToMeters(3)));
    bottom_encoder.setPositionConversionFactor((Math.PI * Units.inchesToMeters(3)));
    top_encoder.setVelocityConversionFactor((Math.PI * Units.inchesToMeters(3) / 60.0));
    bottom_encoder.setVelocityConversionFactor((Math.PI * Units.inchesToMeters(3) / 60.0));
    
    top_pid = top_motor.getPIDController();
    bot_pid = bottom_motor.getPIDController();
    top_pid.setP(0);
    top_pid.setD(0);
    top_pid.setFF(0);
    top_pid.setI(0);

    bot_pid.setP(0);
    bot_pid.setD(0);
    bot_pid.setFF(0);
    bot_pid.setI(0);
  }

  private void topVoltageDrive(Measure<Voltage> voltage) {
    top_motor.setVoltage(voltage.in(Volts));
  }

  private void bottomVoltageDrive(Measure<Voltage> voltage) {
    bottom_motor.setVoltage(voltage.in(Volts));
  }

  public double deadzone(double value, double deadzone) {
    if (Math.abs(value) < deadzone)
      return 0.0;

    return value;
  }

  public void runMotors(CommandXboxController joystick) {
    double Jpower = deadzone(joystick.getRawAxis(1), 0.05);
    if (joystick.rightBumper().getAsBoolean()) {
      top_motor.set(SmartDashboard.getNumber("Top Motor", 0.0));
      bottom_motor.set(SmartDashboard.getNumber("Down Motor", 0.0));
    } else if (joystick.leftBumper().getAsBoolean()) {
      top_pid.setReference(SmartDashboard.getNumber("Top Motor", 0.0), ControlType.kVelocity, 0, topKs);
      // TODO add bottom
    } else {
      top_motor.set(Jpower);
      bottom_motor.set(Jpower);
    }
  }

  // public Command bottomSysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return bottomRoutine.quasistatic(direction);
  // }

  public Command topSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return topRoutine.quasistatic(direction);
  }

  // public Command bottomSysIdDynamic(SysIdRoutine.Direction direction) {
  //   return bottomRoutine.dynamic(direction);
  // }

  public Command topSysIdDynamic(SysIdRoutine.Direction direction) {
    return topRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Encoder", top_encoder.getPosition());
    SmartDashboard.putNumber("Top Velocity", top_encoder.getVelocity());
    SmartDashboard.putNumber("Encoder Counts Per Revolution", top_encoder.getCountsPerRevolution());
    SmartDashboard.putNumber("Bot Encoder", bottom_encoder.getPosition());
    SmartDashboard.putNumber("Bot Velocity", bottom_encoder.getVelocity());
  }
}
