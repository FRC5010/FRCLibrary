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
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class AutoMaps {
    protected HashMap<String,Command> eventMap;
    protected Map<String,List<PathPlannerTrajectory>> paths;    

    public AutoMaps() {   
        eventMap = new HashMap<String,Command>();
        paths = new HashMap<String,List<PathPlannerTrajectory>>();
        // PathPlannerServer.startServer(5811);
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
