package frc.robot.shuffleboardWrapper;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.shuffleboardWrapper.ShuffleBoardSingleton.ShuffleboardUpdateRate;

public class ShuffleboardFloat
{
    protected float value_;
    protected final String name_;
    protected final String tab_;
    protected final ShuffleboardUpdateRate updateRate_;
    protected final SimpleWidget widget_;
    
    public ShuffleboardFloat(final float defaultValue, final String name) { this(defaultValue, name, "Unsorted", ShuffleboardUpdateRate.NONE); }
    public ShuffleboardFloat(final float defaultValue, final String name, final String tab) { this(defaultValue, name, tab, ShuffleboardUpdateRate.NONE); }
    public ShuffleboardFloat(final float defaultValue, final String name, final String tab, final ShuffleboardUpdateRate updateRate)
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
        value_ = widget_.getEntry().getFloat(value_);
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

    public float getValue()
    {
        return value_;
    }

    public void setValue(float value)
    {
        value_ = value;
        widget_.getEntry().setFloat(value_);
    }
}
