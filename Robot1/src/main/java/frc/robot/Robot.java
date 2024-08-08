// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.shuffleboardWrapper.ShuffleBoardSingleton;
import frc.robot.shuffleboardWrapper.ShuffleboardAngle;
import frc.robot.shuffleboardWrapper.ShuffleboardBoolean;
import frc.robot.shuffleboardWrapper.ShuffleboardDouble;
import frc.robot.shuffleboardWrapper.ShuffleboardFloat;
import frc.robot.shuffleboardWrapper.ShuffleboardLength;
import frc.robot.shuffleboardWrapper.ShuffleboardLong;
import frc.robot.shuffleboardWrapper.ShuffleboardString;
import frc.robot.shuffleboardWrapper.ShuffleboardTime;
import frc.robot.shuffleboardWrapper.ShuffleBoardSingleton.ShuffleboardUpdateRate;
import frc.robot.units.Angle.AngleUnit;
import frc.robot.units.Length.LengthUnit;
import frc.robot.units.Time.TimeUnit;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public final String TAB = "Test";

  ShuffleboardLong inputLong = new ShuffleboardLong(0, "Input Long", TAB, ShuffleboardUpdateRate.SLOW);
  ShuffleboardLong outputLong = new ShuffleboardLong(0, "Output Long", TAB, ShuffleboardUpdateRate.NONE);
  ShuffleboardDouble inputDouble = new ShuffleboardDouble(0.0, "Input Double", TAB, ShuffleboardUpdateRate.SLOW);
  ShuffleboardDouble outputDouble = new ShuffleboardDouble(0.0, "Output Double", TAB, ShuffleboardUpdateRate.NONE);
  ShuffleboardFloat inputFloat = new ShuffleboardFloat(0, "Input Float", TAB, ShuffleboardUpdateRate.SLOW);
  ShuffleboardFloat outputFloat = new ShuffleboardFloat(0, "Output Float", TAB, ShuffleboardUpdateRate.NONE);
  ShuffleboardBoolean inputBoolean = new ShuffleboardBoolean(false, "Input Boolean", TAB, ShuffleboardUpdateRate.SLOW);
  ShuffleboardBoolean outputBoolean = new ShuffleboardBoolean(false, "Output Boolean", TAB, ShuffleboardUpdateRate.NONE);
  ShuffleboardString inputString = new ShuffleboardString("none", "Input String", TAB, ShuffleboardUpdateRate.SLOW);
  ShuffleboardString outputString = new ShuffleboardString("none", "Output String", TAB, ShuffleboardUpdateRate.NONE);
  ShuffleboardLength inputLength = new ShuffleboardLength(LengthUnit.METER, 0.0, "Input Length", TAB, ShuffleboardUpdateRate.SLOW);
  ShuffleboardLength outputLength = new ShuffleboardLength(LengthUnit.YARD, 0.0, "Output Length", TAB, ShuffleboardUpdateRate.NONE);
  ShuffleboardTime inputTime = new ShuffleboardTime(TimeUnit.SECOND, 0.0, "Input Time", TAB, ShuffleboardUpdateRate.SLOW);
  ShuffleboardTime outputTime = new ShuffleboardTime(TimeUnit.MILLISECOND, 0.0, "Output Time", TAB, ShuffleboardUpdateRate.NONE);
  ShuffleboardAngle inputAngle = new ShuffleboardAngle(AngleUnit.TURN, 0.0, "Input Angle", TAB, ShuffleboardUpdateRate.SLOW);
  ShuffleboardAngle outputAngle = new ShuffleboardAngle(AngleUnit.DEGREE, 0.0, "Output Angle", TAB, ShuffleboardUpdateRate.NONE);

  int count = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    ShuffleBoardSingleton.getInstance().initialize(this);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    outputLong.setValue(inputLong.getValue());
    outputDouble.setValue(inputDouble.getValue());
    outputFloat.setValue(inputFloat.getValue());
    outputBoolean.setValue(inputBoolean.getValue());
    outputString.setValue(inputString.getValue());
    outputLength.setYards(inputLength.multiply(2.0).getYards());
    outputTime.setTime(inputTime.divide(2.0));
    outputAngle.setAngle(inputAngle.multiply(2.0));

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
