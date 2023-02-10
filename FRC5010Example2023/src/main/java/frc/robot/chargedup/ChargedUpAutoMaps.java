// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.FRC5010.commands.AutoMaps;

/** Add your docs here. */
public class ChargedUpAutoMaps extends AutoMaps{

    public static String blueCone8Start = "Blue Cone 5 Start";

    public void loadAutoPaths(){
        List<PathPlannerTrajectory> examplePath = PathPlanner.loadPathGroup(blueCone8Start, new PathConstraints(1,1));
        paths.put(blueCone8Start, examplePath);
    }
}
