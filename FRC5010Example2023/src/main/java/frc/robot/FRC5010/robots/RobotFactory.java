// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.robots;

import java.util.HashMap;
import java.util.Map;

import frc.robot.FRC5010.constants.Persisted;

/** Add your docs here. */
public class RobotFactory extends RobotConfig {
    public static String WHO_AM_I = "WhoAmI";
    private Persisted<String> whoAmI;
    protected Map<String, Object> robotParts = new HashMap<>();

    // Robot types
    public static class Robots {
        public static final String COMP_BOT_2023 = "2023CompBot";
        public static final String BABY_SWERVE = "BabySwerve";
        public static final String PRACTICE_BOT = "PracticeBot";
        public static final String CURTS_LAPTOP_SIM = "CurtsLaptop";
    }
    //Part types
    public static class Parts {
        public static final String VISION = "Vision";
        public static final String DRIVE = "Drive";
        public static final String AUTO = "Auto"; 
    }
    
    public RobotFactory() {
        // TODO: Figure out a dynamic way to set this such as checking DIO ports
        whoAmI = new Persisted<>(WHO_AM_I, String.class);
        String whichRobot = whoAmI.get();

        switch(whichRobot) {
            case Robots.COMP_BOT_2023: {
                break;
            }
            case Robots.BABY_SWERVE: {
                robotParts = new BabySwerve().getParts();
                break;
            }
            case Robots.PRACTICE_BOT: {
                robotParts = new PracticeBot().getParts();
                break;
            }
            case Robots.CURTS_LAPTOP_SIM: {
                robotParts = new CurtsLaptopSimulator().getParts();
                break;
            }
            default: {
                robotParts = new DefaultRobot().getParts();
                break;
            }
        }
    }

    public Map<String, Object> getParts() {
        return robotParts;
    }
}
