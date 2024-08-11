
package frc.robot.deprecated;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotStateSingleton
{
    // Types
    enum RobotState{
        COMPETITION,
        DEMONSTRATION,
        CALIBRATION,
        DEBUGGING,
        SYSTEM_CHECK
    }
    
    // Private
    private static RobotStateSingleton instance_ = null;
    private RobotState state_ = RobotState.COMPETITION;

    SendableChooser<RobotState> chooser = new SendableChooser<>();

    private RobotStateSingleton()
    {
        chooser.setDefaultOption("Competition", RobotState.COMPETITION);
        chooser.addOption("Demonstration", RobotState.DEMONSTRATION);

        Shuffleboard.getTab("Unsorted").add("Set Robot State", chooser);
        Shuffleboard.getTab("Unsorted").add("Current Robot State", state_.toString());
    }


    // Public
    public static synchronized RobotStateSingleton getInstance()
    {
        if (instance_ == null)
        {
            instance_ = new RobotStateSingleton();
        }

        return instance_;
    }

    public synchronized void setState(RobotState state)
    {
        state_ = state;
    }

    public synchronized RobotState getState()
    {
        return state_;
    }
}
