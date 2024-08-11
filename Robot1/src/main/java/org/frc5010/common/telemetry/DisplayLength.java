
// Package
package org.frc5010.common.telemetry;

// Standard Imports
import java.util.EnumSet;

// FRC5010 Imports
import org.frc5010.common.units.Length;

// WPI Imports
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DisplayLength extends Length{
    protected final String name_;
    protected final String table_;
    protected final DoubleTopic topic_;
    protected final DoublePublisher publisher_;
    protected final DoubleSubscriber subscriber_;
    protected int listenerHandle_;

     // Constructor
    public DisplayLength(final LengthUnit unit, final double unitLength, final String name, final String table)
    {
        super(unit, unitLength);
        name_ = String.format("%s (%s)", name, unit_.getShorthand());
        table_ = table;
        topic_ = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
        publisher_ = topic_.publish();
        subscriber_ = topic_.subscribe(unit_.fromMillimeters(millimeters_));
        listenerHandle_ = NetworkTableInstance.getDefault().addListener(subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> { setLength(unit_, event.valueData.value.getDouble(), false); });

        publisher_.setDefault(unit_.fromMillimeters(millimeters_));
    }

    // Setters
    @Override
    public void setLength(final LengthUnit unit, final double unitLength) { setLength(unit, unitLength, true); }
    public void setLength(final LengthUnit unit, final double unitLength, final boolean publish)
    {
        super.setLength(unit, unitLength);
        if(publish)
        {
            publisher_.set(unit_.fromMillimeters(millimeters_));
        }
    }
}
