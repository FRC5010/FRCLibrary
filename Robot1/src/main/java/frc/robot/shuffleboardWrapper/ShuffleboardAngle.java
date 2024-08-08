
// Package
package frc.robot.shuffleboardWrapper;

// WPI Imports
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

// 5010 Common Imports
import frc.robot.ShuffleBoardSingleton;
import frc.robot.ShuffleBoardSingleton.ShuffleboardUpdateRate;
import frc.robot.units.Angle;

public class ShuffleboardAngle extends Angle {
    // Variables
    protected final String name_;
    protected final String tab_;
    protected final ShuffleboardUpdateRate updateRate_;
    protected final SimpleWidget widget_;

    // Constructor
    public ShuffleboardAngle(final AngleUnit unit, final double unitAngle, final String name, final String tab, final ShuffleboardUpdateRate updateRate)
    {
        super(unit, unitAngle);
        name_ = String.format("%s (%s)", name, unit_.getShorthand());
        tab_ = tab;
        updateRate_ = updateRate;
        widget_ = Shuffleboard.getTab(tab_).add(name_, unit_.fromDegrees(degrees_));

        ShuffleBoardSingleton.getInstance().addToUpdateList(() -> { updateFromShuffleboard(); }, updateRate_);
    }

    // Setters
    @Override
    public void setAngle(final AngleUnit unit, final double unitAngle)
    {
        super.setAngle(unit, unitAngle);
        widget_.getEntry().setDouble(unit_.fromDegrees(degrees_));
    }

    // Update
    public void updateFromShuffleboard()
    {
        degrees_ = unit_.toDegrees(widget_.getEntry().getDouble(unit_.fromDegrees(degrees_)));
    }
}
