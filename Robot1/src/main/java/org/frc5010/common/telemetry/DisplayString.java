
// Package
package org.frc5010.common.telemetry;

// Standard Imports
import java.util.EnumSet;

// WPI Imports
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DisplayString {
    // Variables
    protected String value_;
    protected final String name_;
    protected final String table_;
    protected final StringTopic topic_;
    protected final StringPublisher publisher_;
    protected final StringSubscriber subscriber_;
    protected int listenerHandle_;

    // Constructor
    public DisplayString(final String defaultValue, final String name, final String table)
    {
        value_ = defaultValue;
        name_ = name;
        table_ = table;
        topic_ = NetworkTableInstance.getDefault().getTable(table_).getStringTopic(name_);
        publisher_ = topic_.publish();
        subscriber_ = topic_.subscribe(value_);
        listenerHandle_ = NetworkTableInstance.getDefault().addListener(subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> { setValue(event.valueData.value.getString(), false); });

        publisher_.setDefault(value_);
    }

    // Getters
    public synchronized String getValue()
    {
        return value_;
    }

    // Setters
    public synchronized void setValue(final String value) { setValue(value, true); }
    public synchronized void setValue(final String value, final boolean publish)
    {
        value_ = value;
        if(publish)
        {
            publisher_.set(value_);
        }
    }
}
