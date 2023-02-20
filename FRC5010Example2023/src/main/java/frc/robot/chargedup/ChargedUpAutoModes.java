// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.FRC5010.commands.AutoModes;

/** Add your docs here. */
public class ChargedUpAutoModes extends AutoModes{

    public static String blueCone8Start = "Blue Cone 8 Start";

    public void loadAutoPaths(){
        addPath("Blue Cone 8 Start", new PathConstraints(4, 3));
    }
}
