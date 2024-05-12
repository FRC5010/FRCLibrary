// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.motors.MotorController5010;
import org.frc5010.common.sensors.encoder.GenericEncoder;
import org.frc5010.common.sensors.encoder.SimulatedEncoder;

public class CheeseStick extends SubsystemBase {
  private MotorController5010 stick;
  private GenericPID PID;
  private Mechanism2d mech2d;
  private SimulatedEncoder stickSimEncoder = new SimulatedEncoder(20, 21);
  private SingleJointedArmSim stickSim;
  private MechanismRoot2d simRoot;
  private MechanismLigament2d simCheeseStick;
  private GenericEncoder stickEncoder;

  /** Creates a new CheeseStick. */
  public CheeseStick(MotorController5010 stick, GenericPID PID, Mechanism2d mech2d) {

    this.stick = stick.invert(true);

    this.PID = PID;
    this.mech2d = mech2d;
    this.simRoot = mech2d.getRoot("CheeseStick", 50, 50);
    this.simCheeseStick = new MechanismLigament2d("Cheese Stick", 10, 0);
    simRoot.append(simCheeseStick);
    stickEncoder = stick.getMotorEncoder();
    stickEncoder.setPosition(0);
    stickEncoder.setInverted(true);
    stickEncoder.setPositionConversion(14.33);
  }

  public boolean rotateToSetPoint(double position) {
    double current = stickEncoder.getPosition();
    double error = (position - current);
    double power = 0;
    if (position == 90 && current < 85) {
      power = 0.25;
    } else if (position == 0 && current > 5) {
      power = -0.25;
    } else {
      power = 0;
    }

    if (Math.abs(error) < 20) {
      power *= 0.2;
    }

    stick.set(power);
    return 0 == power;
  }

  public void stop() {
    stick.set(0);
  }

  public double getPosition() {
    return stickEncoder.getPosition();
  }

  @Override

  public void periodic() {
    simCheeseStick.setAngle(stickEncoder.getPosition()); // stand in

  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    stickSim.setInput(stick.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    stickSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    stickSimEncoder.setPosition(Units.radiansToDegrees(stickSim.getAngleRads()));
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(stickSim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    SmartDashboard.putNumber("Stick Sim Rotation", Units.radiansToDegrees(stickSim.getAngleRads()));
  }
}
