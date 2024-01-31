// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>
 * Joystick analog values range from -1 to 1 and motor controller inputs also
 * range from -1 to 1
 * making it easy to work together.
 *
 * <p>
 * In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final int kJoystickPort = 0;

  private CommandXboxController joystick;

  private Shooter shooter = new Shooter();

  @Override
  public void robotInit() {

    joystick = new CommandXboxController(kJoystickPort);
    shooter.setDefaultCommand(new FunctionalCommand(
        () -> shooter.invertMotors(true), // init
        () -> shooter.runMotors(joystick), // execute
        (Boolean interupted) -> { // end
          shooter.stopMotors();
        },
        () -> { // isFinished\
          return false;
        }, shooter));
    // Top Motor Analysis Code
    joystick.a().onTrue(
        new InstantCommand(() -> shooter.invertMotors(false))
        .andThen(shooter.topSysIdQuasistatic(SysIdRoutine.Direction.kForward))
        .andThen(new WaitCommand(7))
        .andThen(shooter.topSysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        .andThen(new WaitCommand(7))
        .andThen(shooter.topSysIdDynamic(SysIdRoutine.Direction.kForward))
        .andThen(new WaitCommand(7))
        .andThen(shooter.topSysIdDynamic(SysIdRoutine.Direction.kReverse)));

    joystick.b().whileTrue(shooter.topSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    joystick.x().whileTrue(shooter.topSysIdDynamic(SysIdRoutine.Direction.kForward));
    joystick.y().whileTrue(shooter.topSysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.

  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {

  }
}
