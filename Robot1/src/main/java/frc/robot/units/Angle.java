
// Package
package frc.robot.units;

public class Angle {
    public enum AngleUnit
    {
        DEGREE("deg", 1.0),
        RADIAN("rad", 180.0 / Math.PI),
        TURN("turn", 360.0);

        private final String shorthand_;
        private final double conversionRate_;

        private AngleUnit(final String shorthand, final double conversionRate)
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

        public double toDegrees(final double unitAngle)
        {
            return unitAngle * conversionRate_;
        }

        public double fromDegrees(final double degrees)
        {
            return degrees / conversionRate_;
        }
    }

    // Variables
    protected final AngleUnit unit_;
    protected double degrees_;

    // Constructor
    public static Angle Degree(final double degrees) { return new Angle(AngleUnit.DEGREE, degrees); }
    public static Angle Radian(final double radians) { return new Angle(AngleUnit.RADIAN, radians); }
    public static Angle Turn(final double turns) { return new Angle(AngleUnit.TURN, turns); }
    public Angle(final AngleUnit unit, final double unitAngle)
    {
        unit_ = unit;
        degrees_ = unit_.toDegrees(unitAngle);
    }

    // Setters
    public void setDegrees(final double degrees) { setAngle(AngleUnit.DEGREE, degrees); }
    public void setRadians(final double radians) { setAngle(AngleUnit.RADIAN, radians); }
    public void setTurns(final double turns) { setAngle(AngleUnit.TURN, turns); }
    public void setAngle(final Angle time) { setAngle(AngleUnit.DEGREE, time.getDegrees()); }
    public void setAngle(final AngleUnit unit, final double unitAngle)
    {
        degrees_ = unit.toDegrees(unitAngle);
    }

    // Getters
    public double getDegrees() { return getAngle(AngleUnit.DEGREE); }
    public double getSeconds() { return getAngle(AngleUnit.RADIAN); }
    public double getMinutes() { return getAngle(AngleUnit.TURN); }
    public double getAngle(final AngleUnit unit)
    {
        return unit.fromDegrees(degrees_);
    }

    // Operations
    public Angle add(final Angle angle)
    {
        return Angle.Degree(degrees_ + angle.getDegrees());
    }

    public Angle subtract(final Angle angle)
    {
        return Angle.Degree(degrees_ - angle.getDegrees());
    }

    public Angle multiply(final double scalar)
    {
        return Angle.Degree(degrees_ * scalar);
    }

    public Angle divide(final double scalar)
    {
        return Angle.Degree(degrees_ / scalar);
    }
}
