
package frc.robot.deprecated;

// WPI Includes
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.ShuffleBoardSingleton;
import frc.robot.ShuffleBoardSingleton.ShuffleboardUpdateRate;
import edu.wpi.first.networktables.GenericEntry;

public class ShuffleboardWrapper<Type>
{
    // Private
    private Type value_;
    private final String name_;
    private final String tabName_;
    private GenericEntry entry_;

    // Public
    public ShuffleboardWrapper(Type defaultValue, String name)
    {
        this(defaultValue, name, "Unsorted", ShuffleboardUpdateRate.NONE);
    }

    public ShuffleboardWrapper(Type defaultValue, String name, String tabName, ShuffleboardUpdateRate updateRate)
    {
        value_ = defaultValue;
        name_ = name;
        tabName_ = tabName;

        // TODO: Add title is bad try catch
        entry_ = Shuffleboard.getTab(tabName_).add(name_, value_).getEntry();
        ShuffleBoardSingleton.getInstance().addToUpdateList(() -> { updateFromShuffleboard(); }, updateRate);
    }

    @SuppressWarnings("unchecked")
    public void updateFromShuffleboard()
    {
        if(entry_.get().isValid())
        {
            value_ = (Type)entry_.get().getValue();
        }
    }


    public Type getValue()
    {
        return value_;
    }

    public void setValue(Type value)
    {
        value_ = value;
        entry_.setValue(value_);
    }
}
