
// Package
package org.frc5010.common.telemetry;

// Standard Imports
import java.util.EnumSet;

// FRC5010 Imports
import org.frc5010.common.units.Time;

// WPI Imports
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DisplayTime extends Time{
    protected final String name_;
    protected final String table_;
    protected final DoubleTopic topic_;
    protected final DoublePublisher publisher_;
    protected final DoubleSubscriber subscriber_;
    protected int listenerHandle_;

     // Constructor
    public DisplayTime(final TimeUnit unit, final double unitTime, final String name, final String table)
    {
        super(unit, unitTime);
        name_ = String.format("%s (%s)", name, unit_.getShorthand());
        table_ = table;
        topic_ = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
        publisher_ = topic_.publish();
        subscriber_ = topic_.subscribe(unit_.fromMilliseconds(milliseconds_));
        listenerHandle_ = NetworkTableInstance.getDefault().addListener(subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> { setTime(unit_, event.valueData.value.getDouble(), false); });

        publisher_.setDefault(unit_.fromMilliseconds(milliseconds_));
    }

    // Setters
    @Override
    public void setTime(final TimeUnit unit, final double unitTime) { setTime(unit, unitTime, true); }
    public void setTime(final TimeUnit unit, final double unitTime, final boolean publish)
    {
        super.setTime(unit, unitTime);
        if(publish)
        {
            publisher_.set(unit_.fromMilliseconds(milliseconds_));
        }
    }
}
