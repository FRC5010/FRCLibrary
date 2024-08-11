
// Package
package org.frc5010.common.telemetry;

// Standard Imports
import java.util.EnumSet;

// WPI Imports
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DisplayLong {
    // Variables
    protected long value_;
    protected final String name_;
    protected final String table_;
    protected final IntegerTopic topic_;
    protected final IntegerPublisher publisher_;
    protected final IntegerSubscriber subscriber_;
    protected int listenerHandle_;

    // Constructor
    public DisplayLong(final long defaultValue, final String name, final String table)
    {
        value_ = defaultValue;
        name_ = name;
        table_ = table;
        topic_ = NetworkTableInstance.getDefault().getTable(table_).getIntegerTopic(name_);
        publisher_ = topic_.publish();
        subscriber_ = topic_.subscribe(value_);
        listenerHandle_ = NetworkTableInstance.getDefault().addListener(subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> { setValue(event.valueData.value.getInteger(), false); });

        publisher_.setDefault(value_);
    }

    // Getters
    public synchronized long getValue()
    {
        return value_;
    }

    // Setters
    public synchronized void setValue(final long value) { setValue(value, true); }
    public synchronized void setValue(final long value, final boolean publish)
    {
        value_ = value;
        if(publish)
        {
            publisher_.set(value_);
        }
    }
}