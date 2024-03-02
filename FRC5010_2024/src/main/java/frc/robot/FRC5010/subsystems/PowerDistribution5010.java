package frc.robot.FRC5010.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.FRC5010.arch.GenericSubsystem;

public class PowerDistribution5010 extends GenericSubsystem {
  private PowerDistribution powerDistribution = new PowerDistribution();
  private static final String UNREGISTERED_CURRENT = "Unregistered Current";
  private static final String TOTAL_CURRENT = "Total Current";
  private static final String PDP_VOLTAGE = "PDP Voltage";
  private static final String AVERAGE_PREFIX = "Average ";

  private Map<String, Integer> channels = new HashMap<>();
  private Map<String, Double> averages = new HashMap<>();
  private Map<String, Integer> counts = new HashMap<>();

  public PowerDistribution5010() {
    declareInitialValues();
  }

  private void declareInitialValues() {
    values.declare(TOTAL_CURRENT, 0.0);
    values.declare(PDP_VOLTAGE, 0.0);
    values.declare(UNREGISTERED_CURRENT, 0.0);
    values.declare(AVERAGE_PREFIX + TOTAL_CURRENT, 0.0);
    values.declare(AVERAGE_PREFIX + PDP_VOLTAGE, 0.0);
  }

  public void registerChannel(String name, int channel) {
    channels.put(name, channel);
    double initialCurrent = powerDistribution.getCurrent(channel);
    values.declare(name, initialCurrent);
    values.declare(AVERAGE_PREFIX + name, 0.0);
  }

  public double getChannelCurrent(String name) {
    return values.getDouble(name);
  }

  private double updateAverage(String name, double value) {
    averages.merge(name, value,
        (avg, newValue) -> (avg * counts.merge(name, 1, Integer::sum) + newValue) / counts.get(name));
    return averages.get(name);
  }

  @Override
  public void periodic() {
    double totalCurrent = powerDistribution.getTotalCurrent();
    double pdpVoltage = powerDistribution.getVoltage();
    double accountedCurrent = 0.0;

    // TODO: Maybe add a verbosity level to the logging...cause it could be a lot
    updatePdpValues(totalCurrent, pdpVoltage);
    updateChannelValues(accountedCurrent);
    values.set(UNREGISTERED_CURRENT, totalCurrent - accountedCurrent);
  }

  private void updatePdpValues(double totalCurrent, double pdpVoltage) {
    values.set(TOTAL_CURRENT, totalCurrent);
    values.set(PDP_VOLTAGE, pdpVoltage);
    values.set(AVERAGE_PREFIX + TOTAL_CURRENT, updateAverage(TOTAL_CURRENT, totalCurrent));
    values.set(AVERAGE_PREFIX + PDP_VOLTAGE, updateAverage(PDP_VOLTAGE, pdpVoltage));
  }

  private void updateChannelValues(double accountedCurrent) {
    for (Map.Entry<String, Integer> entry : channels.entrySet()) {
      double current = powerDistribution.getCurrent(entry.getValue());
      accountedCurrent += current;
      values.set(entry.getKey(), current);
      values.set(AVERAGE_PREFIX + entry.getKey(), updateAverage(entry.getKey(), current));
    }
  }
}
