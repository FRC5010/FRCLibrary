// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import org.frc5010.common.sensors.encoder.GenericEncoder;

/** Add your docs here. */
public class SystemIdentification {
  private static final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private static final MutableMeasure<Angle> m_distance = mutable(Rotations.of(0));
  private static final MutableMeasure<Velocity<Angle>> m_velocity =
      mutable(RotationsPerSecond.of(0));

  private static final MutableMeasure<Angle> m_angle_position = mutable(Rotations.of(0));
  private static final MutableMeasure<Velocity<Angle>> m_anglular_velocity =
      mutable(RotationsPerSecond.of(0));

  public static SysIdRoutine rpmSysIdRoutine(
      MotorController5010 motor,
      GenericEncoder encoder,
      String motorName,
      SubsystemBase subsystemBase) {

    return new SysIdRoutine(
        new Config(Volts.of(1).per(Seconds.of(1)), Volts.of(12), Seconds.of(12.5)),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> voltage) ->
                motor.set(voltage.in(Volts) / RobotController.getBatteryVoltage()),
            log -> {
              log.motor(motorName)
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          motor.get() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_distance.mut_replace(encoder.getPosition(), Rotations))
                  .angularVelocity(
                      m_velocity.mut_replace(encoder.getVelocity(), RotationsPerSecond));
            },
            subsystemBase));
  }

  public static SysIdRoutine angleSysIdRoutine(
      MotorController5010 motor,
      GenericEncoder encoder,
      String motorName,
      SubsystemBase subsystemBase) {

    return new SysIdRoutine(
        new Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> voltage) ->
                motor.set(voltage.in(Volts) / RobotController.getBatteryVoltage()),
            log -> {
              log.motor(motorName)
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          motor.get() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_angle_position.mut_replace(encoder.getPosition(), Degrees))
                  .angularVelocity(
                      m_anglular_velocity.mut_replace(encoder.getVelocity(), DegreesPerSecond));
            },
            subsystemBase));
  }

  public static Command getSysIdQuasistatic(
      SysIdRoutine routine, SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public static Command getSysIdQuasistaticForward(SysIdRoutine routine) {
    return getSysIdQuasistatic(routine, SysIdRoutine.Direction.kForward);
  }

  public static Command getSysIdQuasistaticBackward(SysIdRoutine routine) {
    return getSysIdQuasistatic(routine, SysIdRoutine.Direction.kReverse);
  }

  public static Command getSysIdDynamic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public static Command getSysIdDynamicForward(SysIdRoutine routine) {
    return getSysIdDynamic(routine, SysIdRoutine.Direction.kForward);
  }

  public static Command getSysIdDynamicBackward(SysIdRoutine routine) {
    return getSysIdDynamic(routine, SysIdRoutine.Direction.kReverse);
  }

  public static Command getSysIdFullCommand(
      SysIdRoutine routine, double quasistaticTimeout, double dynamicTimeout, double delay) {
    return getSysIdQuasistaticForward(routine)
        .withTimeout(quasistaticTimeout)
        .andThen(Commands.waitSeconds(delay))
        .andThen(getSysIdQuasistaticBackward(routine).withTimeout(quasistaticTimeout))
        .andThen(Commands.waitSeconds(delay))
        .andThen(getSysIdDynamicForward(routine).withTimeout(dynamicTimeout))
        .andThen(Commands.waitSeconds(delay))
        .andThen(getSysIdDynamicBackward(routine).withTimeout(dynamicTimeout));
  }
}
