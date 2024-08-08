
// Package
package frc.robot.units;

public class Time {
    public enum TimeUnit
    {
        MILLISECOND("msec", 1.0),
        SECOND("sec", 1000.0),
        MINUTE("min", 60000.0),
        HOUR("hr", 3600000.0);

        private final String shorthand_;
        private final double conversionRate_;

        private TimeUnit(final String shorthand, final double conversionRate)
        {
            shorthand_ = shorthand;
            conversionRate_ = conversionRate;
        }

        public String getShorthand()
        {
            return shorthand_;
        }

        public double getConversionRate()
        {
            return conversionRate_;
        }

        public double toMilliseconds(final double unitTime)
        {
            return unitTime * conversionRate_;
        }

        public double fromMilliseconds(final double milliseconds)
        {
            return milliseconds / conversionRate_;
        }
    }

    // Variables
    protected final TimeUnit unit_;
    protected double milliseconds_;

    // Constructor
    public static Time Millisecond(final double milliseconds) { return new Time(TimeUnit.MILLISECOND, milliseconds); }
    public static Time Second(final double seconds) { return new Time(TimeUnit.SECOND, seconds); }
    public static Time Minute(final double minutes) { return new Time(TimeUnit.MINUTE, minutes); }
    public static Time Hour(final double hours) { return new Time(TimeUnit.HOUR, hours); }
    public Time(final TimeUnit unit, final double unitTime)
    {
        unit_ = unit;
        milliseconds_ = unit_.toMilliseconds(unitTime);
    }

    // Setters
    public void setMilliseconds(final double milliseconds) { setTime(TimeUnit.MILLISECOND, milliseconds); }
    public void setSeconds(final double seconds) { setTime(TimeUnit.SECOND, seconds); }
    public void setMinutes(final double minutes) { setTime(TimeUnit.MINUTE, minutes); }
    public void setHours(final double hours) { setTime(TimeUnit.HOUR, hours); }
    public void setTime(final Time time) { setTime(TimeUnit.MILLISECOND, time.getMilliseconds()); }
    public void setTime(final TimeUnit unit, final double unitTime)
    {
        milliseconds_ = unit.toMilliseconds(unitTime);
    }

    // Getters
    public double getMilliseconds() { return getTime(TimeUnit.MILLISECOND); }
    public double getSeconds() { return getTime(TimeUnit.SECOND); }
    public double getMinutes() { return getTime(TimeUnit.MINUTE); }
    public double getHours() { return getTime(TimeUnit.HOUR); }
    public double getTime(final TimeUnit unit)
    {
        return unit.fromMilliseconds(milliseconds_);
    }

    // Operations
    public Time add(final Time time)
    {
        return Time.Millisecond(milliseconds_ + time.getMilliseconds());
    }

    public Time subtract(final Time time)
    {
        return Time.Millisecond(milliseconds_ - time.getMilliseconds());
    }

    public Time multiply(final double scalar)
    {
        return Time.Millisecond(milliseconds_ * scalar);
    }

    public Time divide(final double scalar)
    {
        return Time.Millisecond(milliseconds_ / scalar);
    }
}
