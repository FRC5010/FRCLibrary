// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.FRC5010.GenericMechanism;
import frc.robot.FRC5010.Vision.VisionPhotonMultiCam;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.sensors.GenericGyro;
import frc.robot.FRC5010.sensors.NavXGyro;
import frc.robot.mechanisms.Drive;
import frc.robot.mechanisms.SwerveDriveMech;

/** Add your docs here. */
public class RobotFactory {
    public static String WHO_AM_I = "WhoAmI";
    public Persisted<String> whoAmI;
    private Map<String, Object> robotParts = new HashMap<>();

    // Robot types
    public static class Robots {
        public static final String COMP_BOT_2023 = "2023CompBot";
        public static final String BABY_SWERVE = "BabySwerve";
        public static final String PANCAKE_BOT = "PancakeBot";
        public static final String SIMULATION = "Simulation";
    }
    //Part types
    public static class Parts {
        public static final String VISION = "Vision";
        public static final String DRIVE = "Drive";
    }
    
    public RobotFactory() {
        // TODO: Figure out a dynamic way to set this
        whoAmI = new Persisted<>(WHO_AM_I, Robots.BABY_SWERVE);

        switch(whoAmI.get()) {
            case Robots.COMP_BOT_2023: {
                compBot2023();
                break;
            }
            case Robots.BABY_SWERVE: {
                babySwerve();
                break;
            }
            case Robots.PANCAKE_BOT: {
                pancakeBot();
                break;
            }
            default: {
                simulation();
            }
        }
    }

    public Map<String, Object> getParts() {
        return robotParts;
    }

    public void compBot2023() {

    }

    public void babySwerve() {
        VisionPhotonMultiCam multiVision = new VisionPhotonMultiCam("Vision", 1);
        multiVision.addPhotonCamera("Arducam_OV9281_USB_Camera", 
          new Transform3d( // This describes the vector between the camera lens to the robot center on the ground
            new Translation3d(Units.inchesToMeters(7), 0, Units.inchesToMeters(16.75)), 
            new Rotation3d(0, Units.degreesToRadians(-20), 0)
          )
        );
        GenericGyro gyro = new NavXGyro(SPI.Port.kMXP);
        GenericMechanism drive = new SwerveDriveMech(multiVision);

        //new Drive(multiVision, gyro, Drive.Type.THRIFTY_SWERVE_DRIVE, 12, 12);
        
        robotParts.put(Parts.VISION, multiVision);
        robotParts.put(Parts.DRIVE, drive);
    }

    public void pancakeBot() {

    }

    public void simulation() {

    }
}
