// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    shooter.setDefaultCommand(new RunCommand(() -> shooter.runMotors(joystick), shooter));
    // Top Motor Analysis Code
    joystick.a().whileTrue(shooter.topSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    joystick.b().whileTrue(shooter.topSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    joystick.x().whileTrue(shooter.topSysIdDynamic(SysIdRoutine.Direction.kForward));
    joystick.y().whileTrue(shooter.topSysIdDynamic(SysIdRoutine.Direction.kReverse));
    
    // Bottom Motor Analysis Code
    // joystick.a().whileTrue(shooter.bottomSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // joystick.b().whileTrue(shooter.bottomSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // joystick.x().whileTrue(shooter.bottomSysIdDynamic(SysIdRoutine.Direction.kForward));
    // joystick.y().whileTrue(shooter.bottomSysIdDynamic(SysIdRoutine.Direction.kReverse));
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
