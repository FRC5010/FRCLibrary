// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.constants;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class AutoMaps {
    private HashMap<String,Command> eventMap;
    private Map<String,List<PathPlannerTrajectory>> paths;
    public static String blueCone1Start = "Blue Cone 1 Start";
    

    public AutoMaps() {   
        eventMap = new HashMap<String,Command>();
        paths = new HashMap<String,List<PathPlannerTrajectory>>();
    }

    public void loadAutoPaths() {
        List<PathPlannerTrajectory> examplePath = PathPlanner.loadPathGroup(blueCone1Start, new PathConstraints(1,1));
        paths.put(blueCone1Start, examplePath);
    }
    
    public HashMap<String, Command> getEventMap() {
        return eventMap;
    }

    public Map<String, List<PathPlannerTrajectory>> getPaths() {
        return paths;
    }
    
    public void addMarker(String name, Command command){
        eventMap.put(name, command);
    }

    public void addPath(String name, PathConstraints pathconstraints){
        paths.put(name, PathPlanner.loadPathGroup(name, pathconstraints));
    }
}
