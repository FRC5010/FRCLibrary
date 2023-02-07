// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.mechanisms;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.sensors.Controller;

/** 
 * GenericMechanism should be used as the parent class of any mechanism
 * It enforces the use of the functions:
 *      @function configureButtonBindings(Controller driver, Controller operator)
 *      @function setupDefaultCommands()
 *      @function function initRealOrSim()
 */
public abstract class GenericMechanism {
    protected Mechanism2d mechVisual;
    public GenericMechanism() {
        this.mechVisual = new Mechanism2d(10,10);
    }

    public GenericMechanism(Mechanism2d robotMechVisual) {
        this.mechVisual = robotMechVisual;
    }
    /**
     * configureButtonBindings should map button/axis controls to commands
     * @param driver - driver joystick
     * @param operator - operator joystick
     */
    public abstract void configureButtonBindings(Controller driver, Controller operator);
    
    /**
     * setupDefaultCommands should setup the default commands needed by subsystems
     * It could check for Test mode and enable different commands
     */
    public abstract void setupDefaultCommands();

    /**
     * initRealOrSim should check the real or simulation state of the robot and
     * initialize its code accordingly
     */
    protected abstract void initRealOrSim();

    /**
     * setupPreferences should be implemented in place of using Constants files
     */
    protected void setupPreferences() {

    }

    public abstract Map<String,Command> setAutoCommands(Map<String,List<PathPlannerTrajectory>> paths, HashMap<String, Command> eventMap);
}
