
package frc.robot.deprecated;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;

public class ShuffleBoardSingleton {

    public enum ShuffleboardUpdateRate
    {
        NONE(0.0),
        DEBUG(1.0),
        SLOW(1.0),
        FAST(0.1);

        private final double rate_;

        ShuffleboardUpdateRate(final double rate)
        {
            rate_ = rate;
        }

        public double getRate()
        {
            return rate_;
        }
    }

    // Private
    private static ShuffleBoardSingleton instance_ = null;
    private static final String TAB_NAME = "Debug";

    private ArrayList<Runnable> debugUpdateList = new ArrayList<>();
    private ArrayList<Runnable> slowUpdateList = new ArrayList<>();
    private ArrayList<Runnable> fastUpdateList = new ArrayList<>();

    private ShuffleboardLong debugUpdateCounter_ = null;
    private ShuffleboardLong slowUpdateCounter_ = null;
    private ShuffleboardLong fastUpdateCounter_ = null;
    private ShuffleboardBoolean enableDebugUpdate_ = null;

    private ShuffleBoardSingleton()
    {
    }

    // Public
    public static synchronized ShuffleBoardSingleton getInstance()
    {
        if (instance_ == null)
        {
            instance_ = new ShuffleBoardSingleton();
        }
        return instance_;
    }

    public synchronized void addToUpdateList(Runnable runnable, ShuffleboardUpdateRate updateRate)
    {
        switch (updateRate) {
            case DEBUG:
                debugUpdateList.add(runnable);
                break;
            case SLOW:
                slowUpdateList.add(runnable);
                break;
            case FAST:
                fastUpdateList.add(runnable);
                break;
            case NONE:
            default:
                break;
        }
    }

    public synchronized void initialize(final TimedRobot timedRobot)
    {
        debugUpdateCounter_ = new ShuffleboardLong(0, "Debug Update Counter", TAB_NAME, ShuffleboardUpdateRate.NONE);
        slowUpdateCounter_ = new ShuffleboardLong(0, "Slow Update Counter", TAB_NAME, ShuffleboardUpdateRate.NONE);
        fastUpdateCounter_ = new ShuffleboardLong(0, "Fast Update Counter", TAB_NAME, ShuffleboardUpdateRate.NONE);
        enableDebugUpdate_ = new ShuffleboardBoolean(false, "Enable Debug Update", TAB_NAME, ShuffleboardUpdateRate.SLOW);

        timedRobot.addPeriodic(() -> { ShuffleBoardSingleton.getInstance().debugUpdate(); }, ShuffleboardUpdateRate.DEBUG.getRate());
        timedRobot.addPeriodic(() -> { ShuffleBoardSingleton.getInstance().slowUpdate(); }, ShuffleboardUpdateRate.SLOW.getRate());
        timedRobot.addPeriodic(() -> { ShuffleBoardSingleton.getInstance().fastUpdate(); }, ShuffleboardUpdateRate.FAST.getRate());
    }

    public synchronized void debugUpdate()
    {
        if(enableDebugUpdate_.getValue())
        {
            for (Runnable runnable : debugUpdateList) {
                System.out.println(runnable);
                runnable.run();
            }
            debugUpdateCounter_.setValue(debugUpdateCounter_.getValue() + 1);
        }
    }

    public synchronized void slowUpdate()
    {
        for (Runnable runnable : slowUpdateList) {
            runnable.run();
        }
        slowUpdateCounter_.setValue(slowUpdateCounter_.getValue() + 1);
    }

    public synchronized void fastUpdate()
    {
        for (Runnable runnable : fastUpdateList) {
            runnable.run();
        }
        fastUpdateCounter_.setValue(fastUpdateCounter_.getValue() + 1);
    }
}
