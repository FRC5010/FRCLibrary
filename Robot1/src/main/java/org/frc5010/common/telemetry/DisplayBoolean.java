
// Package
package org.frc5010.common.telemetry;

// Standard Imports
import java.util.EnumSet;

// WPI Imports
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DisplayBoolean {
    // Variables
    protected boolean value_;
    protected final String name_;
    protected final String table_;
    protected final BooleanTopic topic_;
    protected final BooleanPublisher publisher_;
    protected final BooleanSubscriber subscriber_;
    protected int listenerHandle_;

    // Constructor
    public DisplayBoolean(final boolean defaultValue, final String name, final String table)
    {
        value_ = defaultValue;
        name_ = name;
        table_ = table;
        topic_ = NetworkTableInstance.getDefault().getTable(table_).getBooleanTopic(name_);
        publisher_ = topic_.publish();
        subscriber_ = topic_.subscribe(value_);
        listenerHandle_ = NetworkTableInstance.getDefault().addListener(subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> { setValue(event.valueData.value.getBoolean(), false); });

        publisher_.setDefault(value_);
    }

    // Getters
    public synchronized boolean getValue()
    {
        return value_;
    }

    // Setters
    public synchronized void setValue(final boolean value) { setValue(value, true); }
    public synchronized void setValue(final boolean value, final boolean publish)
    {
        value_ = value;
        if(publish)
        {
            publisher_.set(value_);
        }
    }
}
