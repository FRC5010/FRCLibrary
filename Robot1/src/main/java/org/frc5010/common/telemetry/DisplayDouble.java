
// Package
package org.frc5010.common.telemetry;

// Standard Imports
import java.util.EnumSet;

// WPI Imports
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DisplayDouble {
    // Variables
    protected double value_;
    protected final String name_;
    protected final String table_;
    protected final DoubleTopic topic_;
    protected final DoublePublisher publisher_;
    protected final DoubleSubscriber subscriber_;
    protected int listenerHandle_;

    // Constructor
    public DisplayDouble(final double defaultValue, final String name, final String table)
    {
        value_ = defaultValue;
        name_ = name;
        table_ = table;
        topic_ = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
        publisher_ = topic_.publish();
        subscriber_ = topic_.subscribe(value_);
        listenerHandle_ = NetworkTableInstance.getDefault().addListener(subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> { setValue(event.valueData.value.getDouble(), false); });

        publisher_.setDefault(value_);
    }

    // Getters
    public synchronized double getValue()
    {
        return value_;
    }

    // Setters
    public synchronized void setValue(final double value) { setValue(value, true); }
    public synchronized void setValue(final double value, final boolean publish)
    {
        value_ = value;
        if(publish)
        {
            publisher_.set(value_);
        }
    }
}
