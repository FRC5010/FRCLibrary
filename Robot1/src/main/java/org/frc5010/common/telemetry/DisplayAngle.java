
// Package
package org.frc5010.common.telemetry;

// Standard Imports
import java.util.EnumSet;

// FRC5010 Imports
import org.frc5010.common.units.Angle;

// WPI Imports
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DisplayAngle extends Angle{
    protected final String name_;
    protected final String table_;
    protected final DoubleTopic topic_;
    protected final DoublePublisher publisher_;
    protected final DoubleSubscriber subscriber_;
    protected int listenerHandle_;

     // Constructor
    public DisplayAngle(final AngleUnit unit, final double unitAngle, final String name, final String table)
    {
        super(unit, unitAngle);
        name_ = String.format("%s (%s)", name, unit_.getShorthand());
        table_ = table;
        topic_ = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
        publisher_ = topic_.publish();
        subscriber_ = topic_.subscribe(unit_.fromDegrees(degrees_));
        listenerHandle_ = NetworkTableInstance.getDefault().addListener(subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> { setAngle(unit_, event.valueData.value.getDouble(), false); });

        publisher_.setDefault(unit_.fromDegrees(degrees_));
    }

    // Setters
    @Override
    public void setAngle(final AngleUnit unit, final double unitAngle) { setAngle(unit, unitAngle, true); }
    public void setAngle(final AngleUnit unit, final double unitAngle, final boolean publish)
    {
        super.setAngle(unit, unitAngle);
        if(publish)
        {
            publisher_.set(unit_.fromDegrees(degrees_));
        }
    }
}
