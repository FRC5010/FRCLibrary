
// Package
package frc.robot.shuffleboardWrapper;

// WPI Imports
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.shuffleboardWrapper.ShuffleBoardSingleton.ShuffleboardUpdateRate;
import frc.robot.units.Length;

public class ShuffleboardLength extends Length {
    // Variables
    protected final String name_;
    protected final String tab_;
    protected final ShuffleboardUpdateRate updateRate_;
    protected final SimpleWidget widget_;

    // Constructor
    public ShuffleboardLength(final LengthUnit unit, final double unitLength, final String name, final String tab, final ShuffleboardUpdateRate updateRate)
    {
        super(unit, unitLength);
        name_ = String.format("%s (%s)", name, unit_.getShorthand());
        tab_ = tab;
        updateRate_ = updateRate;
        widget_ = Shuffleboard.getTab(tab_).add(name_, unit_.fromMillimeters(millimeters_));

        ShuffleBoardSingleton.getInstance().addToUpdateList(() -> { updateFromShuffleboard(); }, updateRate_);
    }

    // Setters
    @Override
    public void setLength(final LengthUnit unit, final double unitLength)
    {
        super.setLength(unit, unitLength);
        widget_.getEntry().setDouble(unit_.fromMillimeters(millimeters_));
    }

    // Update
    public void updateFromShuffleboard()
    {
        millimeters_ = unit_.toMillimeters(widget_.getEntry().getDouble(unit_.fromMillimeters(millimeters_)));
    }
}
