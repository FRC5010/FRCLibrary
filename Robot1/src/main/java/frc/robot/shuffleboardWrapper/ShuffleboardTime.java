
// Package
package frc.robot.shuffleboardWrapper;

// WPI Imports
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.shuffleboardWrapper.ShuffleBoardSingleton.ShuffleboardUpdateRate;
import frc.robot.units.Time;

public class ShuffleboardTime extends Time {
    // Variables
    protected final String name_;
    protected final String tab_;
    protected final ShuffleboardUpdateRate updateRate_;
    protected final SimpleWidget widget_;

    // Constructor
    public ShuffleboardTime(final TimeUnit unit, final double unitTime, final String name, final String tab, final ShuffleboardUpdateRate updateRate)
    {
        super(unit, unitTime);
        name_ = String.format("%s (%s)", name, unit_.getShorthand());
        tab_ = tab;
        updateRate_ = updateRate;
        widget_ = Shuffleboard.getTab(tab_).add(name_, unit_.fromMilliseconds(milliseconds_));

        ShuffleBoardSingleton.getInstance().addToUpdateList(() -> { updateFromShuffleboard(); }, updateRate_);
    }

    // Setters
    @Override
    public void setTime(final TimeUnit unit, final double unitTime)
    {
        super.setTime(unit, unitTime);
        widget_.getEntry().setDouble(unit_.fromMilliseconds(milliseconds_));
    }

    // Update
    public void updateFromShuffleboard()
    {
        milliseconds_ = unit_.toMilliseconds(widget_.getEntry().getDouble(unit_.fromMilliseconds(milliseconds_)));
    }
}
