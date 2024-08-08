package frc.robot.shuffleboardWrapper;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.ShuffleBoardSingleton;
import frc.robot.ShuffleBoardSingleton.ShuffleboardUpdateRate;

public class ShuffleboardLong
{
    protected long value_;
    protected final String name_;
    protected final String tab_;
    protected final ShuffleboardUpdateRate updateRate_;
    protected final SimpleWidget widget_;
    
    public ShuffleboardLong(final long defaultValue, final String name) { this(defaultValue, name, "Unsorted", ShuffleboardUpdateRate.NONE); }
    public ShuffleboardLong(final long defaultValue, final String name, final String tab) { this(defaultValue, name, tab, ShuffleboardUpdateRate.NONE); }
    public ShuffleboardLong(final long defaultValue, final String name, final String tab, final ShuffleboardUpdateRate updateRate)
    {
        value_ = defaultValue;
        name_ = name;
        tab_ = tab;
        updateRate_ = updateRate;
        widget_ = Shuffleboard.getTab(tab_).add(name_, value_);

        ShuffleBoardSingleton.getInstance().addToUpdateList(() -> { updateFromShuffleboard(); }, updateRate_);
    }

    public void updateFromShuffleboard()
    {
        value_ = widget_.getEntry().getInteger(value_);
    }

    public String getName()
    {
        return name_;
    }

    public String getTab()
    {
        return tab_;
    }

    public ShuffleboardUpdateRate getUpdateRate()
    {
        return updateRate_;
    }

    public long getValue()
    {
        return value_;
    }

    public void setValue(long value)
    {
        value_ = value;
        widget_.getEntry().setInteger(value_);
    }
}
