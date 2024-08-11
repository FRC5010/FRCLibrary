// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5010.common.telemetry.DisplayAngle;
import org.frc5010.common.telemetry.DisplayBoolean;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayFloat;
import org.frc5010.common.telemetry.DisplayLength;
import org.frc5010.common.telemetry.DisplayLong;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayTime;
import org.frc5010.common.units.Angle.AngleUnit;
import org.frc5010.common.units.Length.LengthUnit;
import org.frc5010.common.units.Time.TimeUnit;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
  public final String INPUT_TABLE = "Test/Input";
  public final String OUTPUT_TABLE = "Test/Output";

  DisplayAngle inputAngle = new DisplayAngle(AngleUnit.TURN, 0.0, "Input Angle", INPUT_TABLE);
  DisplayAngle outputAngle = new DisplayAngle(AngleUnit.DEGREE, 0.0, "Output Angle", OUTPUT_TABLE);
  DisplayBoolean inputBoolean = new DisplayBoolean(false, "Input Boolean", INPUT_TABLE);
  DisplayBoolean outputBoolean = new DisplayBoolean(false, "Output Boolean", OUTPUT_TABLE);
  DisplayDouble inputDouble = new DisplayDouble(0.0, "Input Double", INPUT_TABLE);
  DisplayDouble outputDouble = new DisplayDouble(0.0, "Output Double", OUTPUT_TABLE);
  DisplayFloat inputFloat = new DisplayFloat(0, "Input Float", INPUT_TABLE);
  DisplayFloat outputFloat = new DisplayFloat(0, "Output Float", OUTPUT_TABLE);
  DisplayLength inputLength = new DisplayLength(LengthUnit.FOOT, 0.0, "Input Length", INPUT_TABLE);
  DisplayLength outputLength = new DisplayLength(LengthUnit.METER, 0.0, "Output Length", OUTPUT_TABLE);
  DisplayLong inputLong = new DisplayLong(0, "Input Long", INPUT_TABLE);
  DisplayLong outputLong = new DisplayLong(0, "Output Long", OUTPUT_TABLE);
  DisplayString inputString = new DisplayString("default", "Input String", INPUT_TABLE);
  DisplayString outputString = new DisplayString("default", "Output String", OUTPUT_TABLE);
  DisplayTime inputTime = new DisplayTime(TimeUnit.SECOND, 0.0, "Input Time", INPUT_TABLE);
  DisplayTime outputTime = new DisplayTime(TimeUnit.MILLISECOND, 0.0, "Output Time", OUTPUT_TABLE);

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
    
    outputAngle.setAngle(inputAngle);
    outputBoolean.setValue(inputBoolean.getValue());
    outputDouble.setValue(inputDouble.getValue());
    outputFloat.setValue(inputFloat.getValue());
    outputLength.setLength(inputLength);
    outputLong.setValue(inputLong.getValue());
    outputString.setValue(inputString.getValue());
    outputTime.setTime(inputTime);

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
